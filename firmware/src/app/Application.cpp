#include "app/Application.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <math.h>

namespace micro_radar {
namespace {
constexpr uint32_t kTlsCriticalFreeHeap = 52000;
constexpr uint32_t kTlsCriticalMaxBlock = 30000;
constexpr uint32_t kTlsBackgroundFreeHeap = 70000;
constexpr uint32_t kTlsBackgroundMaxBlock = 42000;
constexpr uint32_t kProviderMinFreeHeap = 46000;
constexpr uint32_t kProviderLowHeapBackoffMs = 9000;
constexpr const char* kSetupApSsid = "DesktopAirspace-Setup";

String compactCallsign(String callsign) {
  callsign.trim();
  callsign.replace(" ", "");
  callsign.toUpperCase();
  return callsign;
}

String urlEncode(const String& value) {
  String encoded;
  const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < value.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(value[i]);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.') {
      encoded += static_cast<char>(c);
    } else {
      encoded += '%';
      encoded += hex[c >> 4];
      encoded += hex[c & 0x0F];
    }
  }
  return encoded;
}

String airportCode(JsonObjectConst airport) {
  String code = airport["icao_code"] | "";
  if (code.isEmpty()) code = airport["iata_code"] | "";
  return code;
}

struct RouteCacheEntry {
  String callsign;
  String airline;
  String origin;
  String destination;
  uint32_t updatedMs {0};
  bool valid {false};
};

RouteCacheEntry routeCache[16];
uint8_t routeCacheCursor = 0;

struct AircraftMetaCacheEntry {
  String icao24;
  String registration;
  String typeCode;
  String modelName;
  String operatorName;
  uint32_t updatedMs {0};
  bool valid {false};
};

AircraftMetaCacheEntry aircraftMetaCache[24];
uint8_t aircraftMetaCacheCursor = 0;

bool hasRouteDetails(const Aircraft& aircraft) {
  return aircraft.airlineName.available() && aircraft.routeOrigin.available() && aircraft.routeDestination.available();
}

bool hasAircraftMetadata(const Aircraft& aircraft) {
  return aircraft.typeCode.available() && aircraft.modelName.available();
}

bool tlsHeadroomOk(bool selectedPriority) {
  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t maxBlock = ESP.getMaxAllocHeap();
  const uint32_t minFree = selectedPriority ? kTlsCriticalFreeHeap : kTlsBackgroundFreeHeap;
  const uint32_t minBlock = selectedPriority ? kTlsCriticalMaxBlock : kTlsBackgroundMaxBlock;
  return freeHeap >= minFree && maxBlock >= minBlock;
}

String compactIcao(String icao24) {
  icao24.trim();
  icao24.toUpperCase();
  return icao24;
}

bool routeCacheLookup(const String& key, const Aircraft& source, Aircraft* enrichment) {
  if (!enrichment || key.isEmpty()) return false;
  for (RouteCacheEntry& entry : routeCache) {
    if (!entry.valid || entry.callsign != key) continue;
    enrichment->icao24 = source.icao24;
    enrichment->callsign = source.callsign;
    if (!entry.airline.isEmpty()) enrichment->airlineName = {entry.airline, FieldState::ProviderSupplied};
    if (!entry.origin.isEmpty()) enrichment->routeOrigin = {entry.origin, FieldState::ProviderSupplied};
    if (!entry.destination.isEmpty()) enrichment->routeDestination = {entry.destination, FieldState::ProviderSupplied};
    return true;
  }
  return false;
}

void routeCacheStore(const String& key, const Aircraft& enrichment) {
  if (key.isEmpty()) return;
  RouteCacheEntry* slot = nullptr;
  for (RouteCacheEntry& entry : routeCache) {
    if (entry.valid && entry.callsign == key) {
      slot = &entry;
      break;
    }
  }
  if (!slot) {
    slot = &routeCache[routeCacheCursor++ % (sizeof(routeCache) / sizeof(routeCache[0]))];
  }
  slot->callsign = key;
  slot->airline = enrichment.airlineName.available() ? enrichment.airlineName.value : "";
  slot->origin = enrichment.routeOrigin.available() ? enrichment.routeOrigin.value : "";
  slot->destination = enrichment.routeDestination.available() ? enrichment.routeDestination.value : "";
  slot->updatedMs = millis();
  slot->valid = true;
}

bool aircraftMetaCacheLookup(const String& key, Aircraft* enrichment) {
  if (!enrichment || key.isEmpty()) return false;
  for (AircraftMetaCacheEntry& entry : aircraftMetaCache) {
    if (!entry.valid || entry.icao24 != key) continue;
    enrichment->icao24 = key;
    if (!entry.registration.isEmpty()) enrichment->registration = {entry.registration, FieldState::ProviderSupplied};
    if (!entry.typeCode.isEmpty()) enrichment->typeCode = {entry.typeCode, FieldState::ProviderSupplied};
    if (!entry.modelName.isEmpty()) enrichment->modelName = {entry.modelName, FieldState::ProviderSupplied};
    if (!entry.operatorName.isEmpty()) enrichment->operatorName = {entry.operatorName, FieldState::ProviderSupplied};
    return true;
  }
  return false;
}

void aircraftMetaCacheStore(const String& key, const Aircraft& enrichment) {
  if (key.isEmpty()) return;
  AircraftMetaCacheEntry* slot = nullptr;
  for (AircraftMetaCacheEntry& entry : aircraftMetaCache) {
    if (entry.valid && entry.icao24 == key) {
      slot = &entry;
      break;
    }
  }
  if (!slot) slot = &aircraftMetaCache[aircraftMetaCacheCursor++ % (sizeof(aircraftMetaCache) / sizeof(aircraftMetaCache[0]))];
  slot->icao24 = key;
  slot->registration = enrichment.registration.available() ? enrichment.registration.value : "";
  slot->typeCode = enrichment.typeCode.available() ? enrichment.typeCode.value : "";
  slot->modelName = enrichment.modelName.available() ? enrichment.modelName.value : "";
  slot->operatorName = enrichment.operatorName.available() ? enrichment.operatorName.value : "";
  slot->updatedMs = millis();
  slot->valid = true;
}

bool fetchAircraftMetadata(const Aircraft& source, Aircraft* enrichment, uint16_t timeoutMs = 1000) {
  if (!enrichment || source.icao24.isEmpty()) return false;
  const String icao24 = compactIcao(source.icao24);
  if (icao24.length() < 4) return false;

  WiFiClientSecure* client = new WiFiClientSecure();
  HTTPClient* http = new HTTPClient();
  if (!client || !http) {
    delete client;
    delete http;
    return false;
  }
  client->setInsecure();
  http->setTimeout(timeoutMs);
  http->setReuse(false);
  http->useHTTP10(true);
  http->setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const String url = "https://api.adsbdb.com/v0/aircraft/" + icao24;
  if (!http->begin(*client, url)) {
    delete http;
    delete client;
    return false;
  }
  const int code = http->GET();
  if (code != 200) {
    Serial.printf("Aircraft meta HTTP %d: %s\n", code, icao24.c_str());
    http->end();
    delete http;
    delete client;
    return false;
  }
  Stream& stream = http->getStream();
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, stream);
  http->end();
  delete http;
  delete client;

  if (err != DeserializationError::Ok) {
    Serial.printf("Aircraft meta JSON failed: %s\n", icao24.c_str());
    return false;
  }
  JsonObject aircraft = doc["response"]["aircraft"].as<JsonObject>();
  if (aircraft.isNull()) {
    Serial.printf("Aircraft meta empty: %s\n", icao24.c_str());
    return false;
  }
  const String typeCode = aircraft["icao_type"] | "";
  const String type = aircraft["type"] | "";
  const String manufacturer = aircraft["manufacturer"] | "";
  const String registration = aircraft["registration"] | "";
  const String owner = aircraft["registered_owner"] | "";
  String model = type;
  if (!manufacturer.isEmpty() && model.indexOf(manufacturer) < 0) model = manufacturer + " " + model;
  model.trim();

  if (typeCode.isEmpty() && model.isEmpty() && registration.isEmpty() && owner.isEmpty()) {
    Serial.printf("Aircraft meta no useful fields: %s\n", icao24.c_str());
    return false;
  }
  enrichment->icao24 = source.icao24;
  if (!registration.isEmpty()) enrichment->registration = {registration, FieldState::ProviderSupplied};
  if (!typeCode.isEmpty()) enrichment->typeCode = {typeCode, FieldState::ProviderSupplied};
  if (!model.isEmpty()) enrichment->modelName = {model, FieldState::ProviderSupplied};
  if (!owner.isEmpty()) enrichment->operatorName = {owner, FieldState::ProviderSupplied};
  Serial.printf("Aircraft meta ok: %s type=%s model=%s\n", icao24.c_str(), typeCode.c_str(), model.c_str());
  return true;
}

const char* wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED: return "connected";
    case WL_NO_SSID_AVAIL: return "ssid-not-found";
    case WL_CONNECT_FAILED: return "connect-failed";
    case WL_CONNECTION_LOST: return "connection-lost";
    case WL_DISCONNECTED: return "disconnected";
    case WL_IDLE_STATUS: return "idle";
    default: return "unknown";
  }
}

bool fetchRouteEnrichment(const Aircraft& source, Aircraft* enrichment, uint16_t timeoutMs = 2500) {
  if (!enrichment || !source.callsign.available()) return false;
  const String callsign = compactCallsign(source.callsign.value);
  if (callsign.length() < 3) return false;

  WiFiClientSecure* client = new WiFiClientSecure();
  HTTPClient* http = new HTTPClient();
  if (!client || !http) {
    delete client;
    delete http;
    return false;
  }
  client->setInsecure();
  http->setTimeout(timeoutMs);
  http->setReuse(false);
  http->useHTTP10(true);
  http->setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const String url = "https://api.adsbdb.com/v0/callsign/" + callsign;
  if (!http->begin(*client, url)) {
    delete http;
    delete client;
    return false;
  }
  const int code = http->GET();
  if (code != 200) {
    Serial.printf("Route enrich HTTP %d: %s\n", code, callsign.c_str());
    http->end();
    delete http;
    delete client;
    return false;
  }
  Stream& stream = http->getStream();
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, stream);
  http->end();
  delete http;
  delete client;

  if (err != DeserializationError::Ok) {
    Serial.printf("Route enrich JSON failed: %s\n", callsign.c_str());
    return false;
  }
  JsonObject route = doc["response"]["flightroute"].as<JsonObject>();
  if (route.isNull()) {
    Serial.printf("Route enrich empty: %s\n", callsign.c_str());
    return false;
  }
  const String airline = route["airline"]["name"] | "";
  const String origin = airportCode(route["origin"].as<JsonObjectConst>());
  const String destination = airportCode(route["destination"].as<JsonObjectConst>());
  if (airline.isEmpty() && origin.isEmpty() && destination.isEmpty()) {
    Serial.printf("Route enrich no useful fields: %s\n", callsign.c_str());
    return false;
  }

  enrichment->icao24 = source.icao24;
  enrichment->callsign = source.callsign;
  if (!airline.isEmpty()) enrichment->airlineName = {airline, FieldState::ProviderSupplied};
  if (!origin.isEmpty()) enrichment->routeOrigin = {origin, FieldState::ProviderSupplied};
  if (!destination.isEmpty()) enrichment->routeDestination = {destination, FieldState::ProviderSupplied};
  Serial.printf("Route enrich ok: %s %s %s>%s\n", callsign.c_str(), airline.c_str(), origin.c_str(), destination.c_str());
  return true;
}

}  // namespace

void Application::begin() {
  networkMutex_ = xSemaphoreCreateMutex();
  Serial.println("Application.begin: config");
  config_.begin();
  const AppConfig cfg = config_.get();

  Serial.println("Application.begin: wifi");
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(150);
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(false, false);
  delay(50);
  if (!cfg.wifiSsid.isEmpty()) {
    WiFi.mode(WIFI_AP_STA);
    const bool apOk = WiFi.softAP(kSetupApSsid);
    Serial.printf("Setup AP %s: %s, IP: %s, MAC: %s\n", kSetupApSsid, apOk ? "started" : "failed", WiFi.softAPIP().toString().c_str(), WiFi.softAPmacAddress().c_str());
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
    Serial.printf("Wi-Fi station starting: %s\n", cfg.wifiSsid.c_str());
  } else {
    WiFi.mode(WIFI_AP);
    const bool apOk = WiFi.softAP(kSetupApSsid);
    Serial.printf("Setup AP %s: %s, IP: %s, MAC: %s\n", kSetupApSsid, apOk ? "started" : "failed", WiFi.softAPIP().toString().c_str(), WiFi.softAPmacAddress().c_str());
  }

  Serial.println("Application.begin: radar");
  radar_.configure(cfg);
  Serial.println("Application.begin: maps");
  maps_.begin();
  Serial.println("Application.begin: providers");
  providers_.begin();
  providers_.configure(cfg.providerMode, cfg.localReceiverUrl);
  Serial.println("Application.begin: ui");
  ui_.begin(&radar_, &config_, &maps_);
  Serial.println("Application.begin: web");
  web_.begin(&config_, &radar_, &maps_, &providers_);

  Serial.println("Application.begin: provider task");
  xTaskCreatePinnedToCore(providerTaskEntry, "provider", 7168, this, 1, &providerTaskHandle_, 0);
  Serial.println("Application.begin: map task");
  xTaskCreatePinnedToCore(mapTaskEntry, "map", 12288, this, 0, &mapTaskHandle_, 1);
  Serial.println("Application.begin: enrichment task");
  xTaskCreatePinnedToCore(enrichmentTaskEntry, "enrich", 10240, this, 0, &enrichmentTaskHandle_, 0);
}

void Application::loop() {
  ui_.tick();
  yield();
}

void Application::providerTaskEntry(void* arg) {
  static_cast<Application*>(arg)->providerTask();
}

void Application::mapTaskEntry(void* arg) {
  static_cast<Application*>(arg)->mapTask();
}

void Application::enrichmentTaskEntry(void* arg) {
  static_cast<Application*>(arg)->enrichmentTask();
}

void Application::mapTask() {
  uint32_t lastAttemptMs = 0;
  uint32_t lastAirportAttemptMs = 0;
  uint32_t lastWifiWaitLogMs = 0;
  double lastLat = 999.0;
  double lastLon = 999.0;
  uint16_t lastRange = 0;
  double lastAirportLat = 999.0;
  double lastAirportLon = 999.0;
  uint16_t lastAirportRange = 0;
  while (true) {
    const AppConfig cfg = config_.get();
    const bool scopeChanged = fabs(cfg.homeLat - lastLat) > 0.000001 || fabs(cfg.homeLon - lastLon) > 0.000001 || cfg.rangeNm != lastRange;
    const uint32_t retryDelayMs = scopeChanged ? 0 : 10000;
    const bool currentMap = maps_.isCurrentForLocation(cfg.homeLat, cfg.homeLon, cfg.rangeNm, cfg.mapStyle);
    const bool needsMap = !maps_.hasActivePackage() || !currentMap;
    const bool airportScopeChanged = fabs(cfg.homeLat - lastAirportLat) > 0.000001 || fabs(cfg.homeLon - lastAirportLon) > 0.000001 ||
                                     cfg.rangeNm != lastAirportRange;
    const bool needsAirportPoints = cfg.mapEnabled && cfg.airportsEnabled && currentMap && maps_.pointCount() == 0 && !cfg.geoapifyApiKey.isEmpty() &&
                                    (airportScopeChanged || millis() - lastAirportAttemptMs >= 60000);
    if (cfg.mapEnabled && needsMap && millis() - lastAttemptMs >= retryDelayMs) {
      maps_.setRefreshing(true);
      if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastWifiWaitLogMs >= 5000) {
          lastWifiWaitLogMs = millis();
          Serial.printf("Map task: waiting for Wi-Fi before map refresh status=%s(%d)\n", wifiStatusName(WiFi.status()),
                        static_cast<int>(WiFi.status()));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      lastAttemptMs = millis();
      lastLat = cfg.homeLat;
      lastLon = cfg.homeLon;
      lastRange = cfg.rangeNm;
      Serial.printf("Map task: refreshing static map active=%d current=%d\n", maps_.hasActivePackage(), currentMap);
      if (networkMutex_ && xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(1500)) != pdTRUE) {
        Serial.println("Map task: network busy, retry later");
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      maps_.retryDownloadForLocation(cfg.homeLat, cfg.homeLon, cfg.rangeNm, cfg.geoapifyApiKey, cfg.mapStyle);
      if (networkMutex_) xSemaphoreGive(networkMutex_);
      maps_.setRefreshing(false);
      Serial.println("Map task: " + maps_.statusText());
    } else if (needsAirportPoints) {
      maps_.setRefreshing(true);
      if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastWifiWaitLogMs >= 5000) {
          lastWifiWaitLogMs = millis();
          Serial.printf("Map task: waiting for Wi-Fi before airport refresh status=%s(%d)\n", wifiStatusName(WiFi.status()),
                        static_cast<int>(WiFi.status()));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      lastAirportLat = cfg.homeLat;
      lastAirportLon = cfg.homeLon;
      lastAirportRange = cfg.rangeNm;
      Serial.println("Map task: refreshing airport points for current map");
      if (networkMutex_ && xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(1500)) != pdTRUE) {
        Serial.println("Map task: network busy, airport retry later");
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      lastAirportAttemptMs = millis();
      maps_.refreshAirportPointsForLocation(cfg.homeLat, cfg.homeLon, cfg.rangeNm, cfg.geoapifyApiKey);
      if (networkMutex_) xSemaphoreGive(networkMutex_);
      maps_.setRefreshing(false);
      Serial.println("Map task: airport points=" + String(maps_.pointCount()));
    } else {
      maps_.setRefreshing(false);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Application::providerTask() {
  uint32_t lastWifiLogMs = 0;
  uint32_t lastReconnectMs = 0;
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      const uint32_t now = millis();
      const AppConfig cfg = config_.get();
      if (now - lastWifiLogMs >= 5000) {
        lastWifiLogMs = now;
        Serial.printf("Wi-Fi wait: status=%s(%d) ssid=%s rssi=%d\n", wifiStatusName(WiFi.status()), static_cast<int>(WiFi.status()),
                      cfg.wifiSsid.isEmpty() ? "<none>" : cfg.wifiSsid.c_str(), WiFi.RSSI());
      }
      const wl_status_t wifiStatus = WiFi.status();
      const uint32_t reconnectDelayMs = wifiStatus == WL_NO_SSID_AVAIL ? 30000 : 15000;
      if (!cfg.wifiSsid.isEmpty() && now - lastReconnectMs >= reconnectDelayMs) {
        lastReconnectMs = now;
        Serial.printf("Wi-Fi reconnect: %s\n", cfg.wifiSsid.c_str());
        WiFi.reconnect();
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    lastWifiLogMs = millis();
    std::vector<Aircraft> aircraft;
    AppConfig cfg = config_.get();
    providers_.configure(cfg.providerMode, cfg.localReceiverUrl);
    const uint16_t rangeNm = radar_.rangeNm();
    const uint32_t pollStartMs = millis();
    const uint32_t freeHeap = ESP.getFreeHeap();
    Serial.printf("Provider poll: %.6f, %.6f, %u NM heap=%u max=%u\n", cfg.homeLat, cfg.homeLon, rangeNm, static_cast<unsigned>(freeHeap),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));
    if (freeHeap < kProviderMinFreeHeap) {
      Serial.printf("Provider poll skipped: low heap keeps web alive free=%u\n", static_cast<unsigned>(freeHeap));
      vTaskDelay(pdMS_TO_TICKS(kProviderLowHeapBackoffMs));
      continue;
    }
    if (networkMutex_ && xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(2500)) != pdTRUE) {
      Serial.println("Provider poll skipped: network busy");
      vTaskDelay(pdMS_TO_TICKS(1200));
      continue;
    }
    const bool providerOk = providers_.fetch({cfg.homeLat, cfg.homeLon}, rangeNm, &aircraft);
    if (networkMutex_) xSemaphoreGive(networkMutex_);
    if (providerOk) {
      const size_t providerCount = aircraft.size();
      if (providerCount == 0 && radar_.visibleAircraftCount() > 0) {
        Serial.println("Provider poll empty; keeping recent radar state");
        vTaskDelay(pdMS_TO_TICKS(2500));
        continue;
      }
      radar_.ingest(std::move(aircraft));
      const ProviderStatus status = providers_.status();
      Serial.printf("Provider poll ok: provider=%u visible=%u groundHidden=%s range=%u ms=%u heap=%u max=%u status=%s\n", static_cast<unsigned>(providerCount),
                    static_cast<unsigned>(radar_.visibleAircraftCount()), cfg.showGroundAircraft ? "off" : "on", rangeNm,
                    static_cast<unsigned>(millis() - pollStartMs), static_cast<unsigned>(ESP.getFreeHeap()), static_cast<unsigned>(ESP.getMaxAllocHeap()),
                    status.message.c_str());
      ui_.invalidate();
      if (enrichmentTaskHandle_) xTaskNotifyGive(enrichmentTaskHandle_);
    } else {
      Serial.println("Provider poll failed");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void Application::enrichmentTask() {
  String lastSelectedAttempt;
  uint32_t lastSelectedAttemptMs = 0;
  uint32_t lastMetadataAttemptMs = 0;
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }
    const auto aircraft = radar_.snapshotVisibleAircraft();
    const Aircraft* selected = nullptr;
    for (const Aircraft& ac : aircraft) {
      if (ac.selected) {
        selected = &ac;
        break;
      }
    }
    const Aircraft* candidate = selected;
    const bool selectedPriority = selected != nullptr;
    if (!candidate) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
      continue;
    }
    const String metaKey = compactIcao(candidate->icao24);
    const bool needsMeta = !hasAircraftMetadata(*candidate) && metaKey.length() >= 4;
    const bool needsRoute = candidate->callsign.available() && !candidate->callsign.value.isEmpty() && !hasRouteDetails(*candidate);
    const String key = candidate->callsign.available() ? compactCallsign(candidate->callsign.value) : "";
    const uint32_t now = millis();
    Aircraft enrichment;
    if (needsRoute && key.length() >= 3 && routeCacheLookup(key, *candidate, &enrichment)) {
      radar_.mergeEnrichment(enrichment);
      ui_.invalidate();
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(80));
      continue;
    }
    if (!needsRoute && needsMeta && aircraftMetaCacheLookup(metaKey, &enrichment)) {
      radar_.mergeEnrichment(enrichment);
      ui_.invalidate();
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(80));
      continue;
    }
    const String throttleKey = needsRoute ? key : metaKey;
    if (throttleKey.length() < 3) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
      continue;
    }
    if (selectedPriority && throttleKey == lastSelectedAttempt && now - lastSelectedAttemptMs < 5000) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(150));
      continue;
    }
    if (!needsRoute && needsMeta && now - lastMetadataAttemptMs < 2500) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
      continue;
    }
    if (needsRoute && !tlsHeadroomOk(true)) {
      Serial.printf("Enrich skip: low TLS heap free=%u max=%u selected=%d\n", static_cast<unsigned>(ESP.getFreeHeap()),
                    static_cast<unsigned>(ESP.getMaxAllocHeap()), 1);
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(700));
      continue;
    }
    if (networkMutex_ && xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(350)) != pdTRUE) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(160));
      continue;
    }
    if (needsRoute && !tlsHeadroomOk(true)) {
      if (networkMutex_) xSemaphoreGive(networkMutex_);
      Serial.printf("Enrich skip after lock: low TLS heap free=%u max=%u selected=%d\n", static_cast<unsigned>(ESP.getFreeHeap()),
                    static_cast<unsigned>(ESP.getMaxAllocHeap()), 1);
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(700));
      continue;
    }
    bool ok = false;
    if (needsRoute) {
      Serial.printf("Route enrich selected attempt: %s\n", key.c_str());
      ok = fetchRouteEnrichment(*candidate, &enrichment, 1800);
      if (!ok) {
        Serial.println("Route enrich unavailable: ADSBDB failed or has no route");
      }
    } else if (needsMeta) {
      Serial.printf("Aircraft meta %s attempt: %s\n", selectedPriority ? "selected" : "prefetch", metaKey.c_str());
      ok = fetchAircraftMetadata(*candidate, &enrichment, 1000);
    }
    if (networkMutex_) xSemaphoreGive(networkMutex_);
    if (selectedPriority) {
      lastSelectedAttempt = throttleKey;
      lastSelectedAttemptMs = millis();
    }
    if (needsMeta) lastMetadataAttemptMs = millis();
    if (ok) {
      if (needsRoute) routeCacheStore(key, enrichment);
      else if (needsMeta) aircraftMetaCacheStore(metaKey, enrichment);
      else routeCacheStore(key, enrichment);
      radar_.mergeEnrichment(enrichment);
      ui_.invalidate();
    }
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(120));
  }
}

}  // namespace micro_radar
