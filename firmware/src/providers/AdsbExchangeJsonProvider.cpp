#include "providers/AdsbExchangeJsonProvider.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

namespace micro_radar {
namespace {
struct RouteCacheEntry {
  String key;
  String airline;
  String origin;
  String destination;
  uint32_t cachedMs {0};
};

RouteCacheEntry routeCache[12];

String jsonValueToString(JsonVariantConst value) {
  if (value.is<const char*>()) return String(value.as<const char*>());
  if (value.is<int>()) return String(value.as<int>());
  if (value.is<unsigned int>()) return String(value.as<unsigned int>());
  if (value.is<long>()) return String(value.as<long>());
  return "";
}

bool jsonHasNumber(JsonVariantConst value) {
  return value.is<int>() || value.is<long>() || value.is<unsigned int>() || value.is<unsigned long>() || value.is<float>() || value.is<double>();
}

String normalizeSquawk(String value) {
  value.trim();
  while (value.length() < 4 && !value.isEmpty()) value = "0" + value;
  return value;
}

String emergencyFromSquawk(const String& squawk) {
  if (squawk == "7700") return "Emergency";
  if (squawk == "7600") return "Radio fail";
  if (squawk == "7500") return "Security";
  return "";
}

String compactCallsign(String callsign) {
  callsign.trim();
  callsign.replace(" ", "");
  callsign.toUpperCase();
  return callsign;
}

String airportCode(JsonObjectConst airport) {
  String code = airport["icao_code"] | "";
  if (code.isEmpty()) code = airport["iata_code"] | "";
  return code;
}

bool cachedRoute(const String& key, Aircraft* aircraft) {
  const uint32_t now = millis();
  for (RouteCacheEntry& entry : routeCache) {
    if (entry.key == key && now - entry.cachedMs < 6UL * 60UL * 60UL * 1000UL) {
      if (!entry.airline.isEmpty()) aircraft->airlineName = {entry.airline, FieldState::ProviderSupplied};
      if (!entry.origin.isEmpty()) aircraft->routeOrigin = {entry.origin, FieldState::ProviderSupplied};
      if (!entry.destination.isEmpty()) aircraft->routeDestination = {entry.destination, FieldState::ProviderSupplied};
      return true;
    }
  }
  return false;
}

void storeRouteCache(const String& key, const String& airline, const String& origin, const String& destination) {
  uint8_t slot = 0;
  uint32_t oldest = routeCache[0].cachedMs;
  for (uint8_t i = 0; i < 12; ++i) {
    if (routeCache[i].key == key || routeCache[i].key.isEmpty()) {
      slot = i;
      break;
    }
    if (routeCache[i].cachedMs < oldest) {
      oldest = routeCache[i].cachedMs;
      slot = i;
    }
  }
  routeCache[slot].key = key;
  routeCache[slot].airline = airline;
  routeCache[slot].origin = origin;
  routeCache[slot].destination = destination;
  routeCache[slot].cachedMs = millis();
}
}  // namespace

AdsbExchangeJsonProvider::AdsbExchangeJsonProvider(String baseUrl, String attribution, uint16_t minPollIntervalSec, ProviderId providerId)
    : baseUrl_(std::move(baseUrl)), attribution_(std::move(attribution)), providerId_(providerId), minPollIntervalSec_(minPollIntervalSec) {}

bool AdsbExchangeJsonProvider::begin() { return true; }
bool AdsbExchangeJsonProvider::configure(const String&) { return true; }
bool AdsbExchangeJsonProvider::validateConfiguration(String*) const { return true; }

String AdsbExchangeJsonProvider::makeRadiusUrl(geo::Location origin, uint16_t radiusNm) const {
  const uint16_t providerRadius = radiusNm > maxRadiusNm_ ? maxRadiusNm_ : radiusNm;
  if (baseUrl_.indexOf("opendata.adsb.fi") >= 0) {
    return baseUrl_ + "/lat/" + String(origin.lat, 6) + "/lon/" + String(origin.lon, 6) + "/dist/" + String(providerRadius);
  }
  return baseUrl_ + "/point/" + String(origin.lat, 6) + "/" + String(origin.lon, 6) + "/" + String(providerRadius);
}

bool AdsbExchangeJsonProvider::fetchAircraft(geo::Location origin, uint16_t radiusNm, std::vector<Aircraft>* out) {
  const String url = makeRadiusUrl(origin, radiusNm);
  const bool secure = url.startsWith("https://");
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  if (secure) {
    secureClient.setInsecure();
  }
  HTTPClient http;
  http.setTimeout(3500);
  http.setReuse(false);
  http.useHTTP10(true);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const bool began = secure ? http.begin(secureClient, url) : http.begin(plainClient, url);
  if (!began) {
    status_.lastError = ProviderError::Network;
    status_.message = "HTTP begin failed";
    return false;
  }
  const int code = http.GET();
  if (code != 200) {
    status_.lastError = mapError(code, "");
    status_.message = "Provider HTTP " + String(code);
    http.end();
    return false;
  }
  Stream& stream = http.getStream();
  const bool ok = parsePayload(stream, origin, radiusNm, out);
  http.end();
  if (ok) {
    status_.lastError = ProviderError::None;
    status_.lastSuccessMs = millis();
    status_.message = "ok";
  }
  return ok;
}

bool AdsbExchangeJsonProvider::parsePayload(Stream& stream, geo::Location origin, uint16_t radiusNm, std::vector<Aircraft>* out) {
  JsonDocument filter;
  JsonObject itemFilter = filter["ac"].add<JsonObject>();
  itemFilter["hex"] = true;
  itemFilter["flight"] = true;
  itemFilter["r"] = true;
  itemFilter["t"] = true;
  itemFilter["desc"] = true;
  itemFilter["ownOp"] = true;
  itemFilter["lat"] = true;
  itemFilter["lon"] = true;
  itemFilter["ground"] = true;
  itemFilter["alt_baro"] = true;
  itemFilter["alt_geom"] = true;
  itemFilter["gs"] = true;
  itemFilter["track"] = true;
  itemFilter["mag_heading"] = true;
  itemFilter["true_heading"] = true;
  itemFilter["baro_rate"] = true;
  itemFilter["squawk"] = true;
  itemFilter["emergency"] = true;
  itemFilter["type"] = true;
  itemFilter["seen"] = true;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, stream, DeserializationOption::Filter(filter));
  if (err) {
    status_.lastError = ProviderError::MalformedJson;
    status_.message = err.c_str();
    return false;
  }
  JsonArray arr = doc["ac"].as<JsonArray>();
  out->clear();
  for (JsonObject item : arr) {
    if (!item["lat"].is<double>() || !item["lon"].is<double>()) continue;
    Aircraft ac;
    ac.icao24 = item["hex"] | "";
    ac.provider = providerId_;
    ac.callsign = {String(item["flight"] | ""), item["flight"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.registration = {String(item["r"] | ""), item["r"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.typeCode = {String(item["t"] | ""), item["t"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.modelName = {String(item["desc"] | ""), item["desc"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.operatorName = {String(item["ownOp"] | ""), item["ownOp"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.latitude = {item["lat"].as<double>(), FieldState::ProviderSupplied};
    ac.longitude = {item["lon"].as<double>(), FieldState::ProviderSupplied};
    const bool onGround = (item["alt_baro"].is<const char*>() && String(item["alt_baro"].as<const char*>()) == "ground") || (item["ground"] | false);
    ac.onGround = {onGround, onGround || item["ground"].is<bool>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.baroAltitudeFt = {onGround ? 0 : (item["alt_baro"] | 0), jsonHasNumber(item["alt_baro"]) || onGround ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.geomAltitudeFt = {item["alt_geom"] | 0, jsonHasNumber(item["alt_geom"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.groundSpeedKt = {item["gs"] | 0.0, jsonHasNumber(item["gs"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.trackDeg = {item["track"] | 0.0, jsonHasNumber(item["track"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.headingDeg = {item["mag_heading"] | (item["true_heading"] | 0.0), jsonHasNumber(item["mag_heading"]) || jsonHasNumber(item["true_heading"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.verticalRateFpm = {item["baro_rate"] | 0, jsonHasNumber(item["baro_rate"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    const String squawk = normalizeSquawk(jsonValueToString(item["squawk"]));
    ac.squawk = {squawk, squawk.isEmpty() ? FieldState::Unavailable : FieldState::ProviderSupplied};
    String emergency = jsonValueToString(item["emergency"]);
    if (emergency.isEmpty()) emergency = emergencyFromSquawk(squawk);
    ac.emergency = {emergency, emergency.isEmpty() ? FieldState::Unavailable : FieldState::ProviderSupplied};
    ac.sourceType = {String(item["type"] | ""), item["type"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.messageAgeSec = {static_cast<uint32_t>(item["seen"] | 0), jsonHasNumber(item["seen"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.lastReceivedMs = {static_cast<uint64_t>(millis()), FieldState::ProviderSupplied};
    geo::Location pos {ac.latitude.value, ac.longitude.value};
    ac.distanceNm = geo::distanceNm(origin, pos);
    if (ac.distanceNm > radiusNm) continue;
    ac.bearingDeg = geo::bearingDeg(origin, pos);
    ac.stale = ac.messageAgeSec.available() && ac.messageAgeSec.value > 30;
    out->push_back(ac);
  }
  return true;
}

bool AdsbExchangeJsonProvider::fetchOptionalRoute(Aircraft* aircraft) {
  if (!aircraft || !aircraft->callsign.available()) return false;
  const String callsign = compactCallsign(aircraft->callsign.value);
  if (callsign.length() < 3) return false;
  if (cachedRoute(callsign, aircraft)) return true;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(2500);
  http.setReuse(false);
  http.useHTTP10(true);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const String url = "https://api.adsbdb.com/v0/callsign/" + callsign;
  if (!http.begin(client, url)) return false;
  const int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }
  Stream& stream = http.getStream();
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, stream);
  http.end();

  if (err != DeserializationError::Ok) return false;
  JsonObject route = doc["response"]["flightroute"].as<JsonObject>();
  if (route.isNull()) return false;
  const String airline = route["airline"]["name"] | "";
  const String originCode = airportCode(route["origin"].as<JsonObjectConst>());
  const String destinationCode = airportCode(route["destination"].as<JsonObjectConst>());
  if (!airline.isEmpty()) aircraft->airlineName = {airline, FieldState::ProviderSupplied};
  if (!originCode.isEmpty()) aircraft->routeOrigin = {originCode, FieldState::ProviderSupplied};
  if (!destinationCode.isEmpty()) aircraft->routeDestination = {destinationCode, FieldState::ProviderSupplied};
  storeRouteCache(callsign, airline, originCode, destinationCode);
  if (!airline.isEmpty() || !originCode.isEmpty() || !destinationCode.isEmpty()) {
    Serial.printf("Route enrich: %s %s %s>%s\n", callsign.c_str(), airline.c_str(), originCode.c_str(), destinationCode.c_str());
    return true;
  }
  return false;
}

ProviderCapabilities AdsbExchangeJsonProvider::getCapabilities() const {
  ProviderCapabilities caps;
  caps.minPollIntervalSec = minPollIntervalSec_;
  caps.attribution = attribution_;
  return caps;
}

ProviderStatus AdsbExchangeJsonProvider::getProviderStatus() const { return status_; }
void AdsbExchangeJsonProvider::cancelPendingRequest() {}

ProviderError AdsbExchangeJsonProvider::mapError(int httpCode, const String&) const {
  if (httpCode == 401 || httpCode == 403) return ProviderError::Auth;
  if (httpCode == 429) return ProviderError::RateLimited;
  if (httpCode <= 0) return ProviderError::Timeout;
  return ProviderError::Http;
}

void AdsbExchangeJsonProvider::shutdown() {}

}  // namespace micro_radar
