#include "providers/LocalDump1090Provider.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClient.h>

namespace micro_radar {
namespace {
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
}  // namespace

bool LocalDump1090Provider::begin() { return true; }

bool LocalDump1090Provider::configure(const String& url) {
  url_ = url;
  url_.trim();
  return true;
}

bool LocalDump1090Provider::configured() const { return url_.startsWith("http://") || url_.startsWith("https://"); }

bool LocalDump1090Provider::validateConfiguration(String* error) const {
  if (url_.isEmpty() || configured()) return true;
  if (error) *error = "Local receiver URL must start with http:// or https://";
  return false;
}

bool LocalDump1090Provider::fetchAircraft(geo::Location origin, uint16_t radiusNm, std::vector<Aircraft>* out) {
  if (!out) return false;
  out->clear();
  if (!configured()) {
    status_.lastError = ProviderError::Unsupported;
    status_.message = "local receiver not configured";
    return false;
  }
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(1200);
  http.setReuse(false);
  http.useHTTP10(true);
  if (!http.begin(client, url_)) {
    status_.lastError = ProviderError::Network;
    status_.message = "local receiver begin failed";
    return false;
  }
  const int code = http.GET();
  if (code != 200) {
    status_.lastError = mapError(code, "");
    status_.message = "Local receiver HTTP " + String(code);
    http.end();
    return false;
  }
  Stream& stream = http.getStream();
  const bool ok = parsePayload(stream, origin, radiusNm, out);
  http.end();
  if (ok) {
    status_.lastError = ProviderError::None;
    status_.lastSuccessMs = millis();
    status_.message = "ok " + String(out->size());
  }
  return ok;
}

bool LocalDump1090Provider::parsePayload(Stream& stream, geo::Location origin, uint16_t radiusNm, std::vector<Aircraft>* out) {
  JsonDocument filter;
  JsonObject aircraftFilter = filter["aircraft"].add<JsonObject>();
  JsonObject acFilter = filter["ac"].add<JsonObject>();
  JsonObject filters[] = {aircraftFilter, acFilter};
  for (JsonObject itemFilter : filters) {
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
    itemFilter["altitude"] = true;
    itemFilter["alt_geom"] = true;
    itemFilter["gs"] = true;
    itemFilter["speed"] = true;
    itemFilter["track"] = true;
    itemFilter["mag_heading"] = true;
    itemFilter["true_heading"] = true;
    itemFilter["baro_rate"] = true;
    itemFilter["vert_rate"] = true;
    itemFilter["squawk"] = true;
    itemFilter["emergency"] = true;
    itemFilter["type"] = true;
    itemFilter["seen"] = true;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, stream, DeserializationOption::Filter(filter));
  if (err) {
    status_.lastError = ProviderError::MalformedJson;
    status_.message = String("local JSON ") + err.c_str();
    return false;
  }
  JsonArray arr = doc["aircraft"].as<JsonArray>();
  if (arr.isNull()) arr = doc["ac"].as<JsonArray>();
  if (arr.isNull()) {
    status_.lastError = ProviderError::MalformedJson;
    status_.message = "local JSON no aircraft";
    return false;
  }
  out->clear();
  for (JsonObject item : arr) {
    if (!jsonHasNumber(item["lat"]) || !jsonHasNumber(item["lon"])) continue;
    Aircraft ac;
    ac.icao24 = String(item["hex"] | "");
    ac.provider = ProviderId::LocalReceiver;
    ac.callsign = {String(item["flight"] | ""), item["flight"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.registration = {String(item["r"] | ""), item["r"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.typeCode = {String(item["t"] | ""), item["t"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.modelName = {String(item["desc"] | ""), item["desc"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.operatorName = {String(item["ownOp"] | ""), item["ownOp"].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.latitude = {item["lat"].as<double>(), FieldState::ProviderSupplied};
    ac.longitude = {item["lon"].as<double>(), FieldState::ProviderSupplied};
    const bool onGround = (item["alt_baro"].is<const char*>() && String(item["alt_baro"].as<const char*>()) == "ground") || (item["altitude"].is<const char*>() && String(item["altitude"].as<const char*>()) == "ground") || (item["ground"] | false);
    ac.onGround = {onGround, onGround || item["ground"].is<bool>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    const int32_t alt = jsonHasNumber(item["alt_baro"]) ? item["alt_baro"].as<int32_t>() : jsonHasNumber(item["altitude"]) ? item["altitude"].as<int32_t>() : 0;
    ac.baroAltitudeFt = {onGround ? 0 : alt, (jsonHasNumber(item["alt_baro"]) || jsonHasNumber(item["altitude"]) || onGround) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.geomAltitudeFt = {item["alt_geom"] | 0, jsonHasNumber(item["alt_geom"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.groundSpeedKt = {jsonHasNumber(item["gs"]) ? item["gs"].as<double>() : jsonHasNumber(item["speed"]) ? item["speed"].as<double>() : 0.0,
                        (jsonHasNumber(item["gs"]) || jsonHasNumber(item["speed"])) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.trackDeg = {item["track"] | 0.0, jsonHasNumber(item["track"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.headingDeg = {item["mag_heading"] | (item["true_heading"] | 0.0), jsonHasNumber(item["mag_heading"]) || jsonHasNumber(item["true_heading"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    if (!ac.headingDeg.available() && ac.trackDeg.available()) ac.headingDeg = ac.trackDeg;
    ac.verticalRateFpm = {item["baro_rate"] | (item["vert_rate"] | 0), jsonHasNumber(item["baro_rate"]) || jsonHasNumber(item["vert_rate"]) ? FieldState::ProviderSupplied : FieldState::Unavailable};
    const String squawk = normalizeSquawk(jsonValueToString(item["squawk"]));
    ac.squawk = {squawk, squawk.isEmpty() ? FieldState::Unavailable : FieldState::ProviderSupplied};
    String emergency = jsonValueToString(item["emergency"]);
    if (emergency.isEmpty()) emergency = emergencyFromSquawk(squawk);
    ac.emergency = {emergency, emergency.isEmpty() ? FieldState::Unavailable : FieldState::ProviderSupplied};
    ac.sourceType = {String(item["type"] | "local"), FieldState::ProviderSupplied};
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

bool LocalDump1090Provider::fetchOptionalRoute(Aircraft*) { return false; }

ProviderCapabilities LocalDump1090Provider::getCapabilities() const {
  ProviderCapabilities caps;
  caps.minPollIntervalSec = 1;
  caps.attribution = "Local dump1090/readsb receiver";
  return caps;
}

ProviderStatus LocalDump1090Provider::getProviderStatus() const { return status_; }
void LocalDump1090Provider::cancelPendingRequest() {}

ProviderError LocalDump1090Provider::mapError(int httpCode, const String&) const {
  if (httpCode == 401 || httpCode == 403) return ProviderError::Auth;
  if (httpCode == 429) return ProviderError::RateLimited;
  if (httpCode <= 0) return ProviderError::Timeout;
  return ProviderError::Http;
}

void LocalDump1090Provider::shutdown() {}

}  // namespace micro_radar
