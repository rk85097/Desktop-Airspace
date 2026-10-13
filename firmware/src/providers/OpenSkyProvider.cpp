#include "providers/OpenSkyProvider.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <math.h>

namespace micro_radar {
namespace {
constexpr double kMetersToFeet = 3.280839895;
constexpr double kMetersPerSecondToKnots = 1.943844492;

String compactCallsign(String callsign) {
  callsign.trim();
  callsign.replace(" ", "");
  callsign.toUpperCase();
  return callsign;
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

bool OpenSkyProvider::begin() { return true; }
bool OpenSkyProvider::configure(const String&) { return true; }
bool OpenSkyProvider::validateConfiguration(String*) const { return true; }

String OpenSkyProvider::makeBoundsUrl(geo::Location origin, uint16_t radiusNm) const {
  const double latDelta = static_cast<double>(radiusNm) / 60.0;
  const double cosLat = max(0.20, fabs(cos(origin.lat * 0.01745329252)));
  const double lonDelta = static_cast<double>(radiusNm) / (60.0 * cosLat);
  return "https://opensky-network.org/api/states/all?lamin=" + String(origin.lat - latDelta, 6) +
         "&lomin=" + String(origin.lon - lonDelta, 6) +
         "&lamax=" + String(origin.lat + latDelta, 6) +
         "&lomax=" + String(origin.lon + lonDelta, 6);
}

bool OpenSkyProvider::fetchAircraft(geo::Location origin, uint16_t radiusNm, std::vector<Aircraft>* out) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(4500);
  http.setReuse(false);
  http.useHTTP10(true);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, makeBoundsUrl(origin, radiusNm))) {
    status_.lastError = ProviderError::Network;
    status_.message = "OpenSky begin failed";
    return false;
  }
  const int code = http.GET();
  if (code != 200) {
    status_.lastError = mapError(code, "");
    status_.message = "OpenSky HTTP " + String(code);
    http.end();
    return false;
  }
  Stream& stream = http.getStream();
  const bool ok = parsePayload(stream, origin, radiusNm, out);
  http.end();
  if (ok) {
    status_.lastError = ProviderError::None;
    status_.lastSuccessMs = millis();
    status_.message = "ok " + String(out ? out->size() : 0);
  }
  return ok;
}

bool OpenSkyProvider::parsePayload(Stream& stream, geo::Location origin, uint16_t radiusNm, std::vector<Aircraft>* out) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, stream);
  if (err) {
    status_.lastError = ProviderError::MalformedJson;
    status_.message = String("OpenSky JSON ") + err.c_str();
    return false;
  }
  JsonArray states = doc["states"].as<JsonArray>();
  out->clear();
  for (JsonArray row : states) {
    if (row.size() < 17 || row[5].isNull() || row[6].isNull()) continue;
    Aircraft ac;
    ac.icao24 = String(row[0] | "");
    ac.callsign = {compactCallsign(String(row[1] | "")), row[1].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.country = {String(row[2] | ""), row[2].is<const char*>() ? FieldState::ProviderSupplied : FieldState::Unavailable};
    ac.longitude = {row[5].as<double>(), FieldState::ProviderSupplied};
    ac.latitude = {row[6].as<double>(), FieldState::ProviderSupplied};
    const bool onGround = row[8] | false;
    ac.onGround = {onGround, FieldState::ProviderSupplied};
    if (!row[7].isNull()) ac.baroAltitudeFt = {static_cast<int32_t>(round(row[7].as<double>() * kMetersToFeet)), FieldState::ProviderSupplied};
    if (!row[13].isNull()) ac.geomAltitudeFt = {static_cast<int32_t>(round(row[13].as<double>() * kMetersToFeet)), FieldState::ProviderSupplied};
    if (!row[9].isNull()) ac.groundSpeedKt = {row[9].as<double>() * kMetersPerSecondToKnots, FieldState::ProviderSupplied};
    if (!row[10].isNull()) {
      ac.trackDeg = {row[10].as<double>(), FieldState::ProviderSupplied};
      ac.headingDeg = ac.trackDeg;
    }
    if (!row[11].isNull()) ac.verticalRateFpm = {static_cast<int32_t>(round(row[11].as<double>() * kMetersToFeet * 60.0)), FieldState::ProviderSupplied};
    const String squawk = normalizeSquawk(String(row[14] | ""));
    ac.squawk = {squawk, squawk.isEmpty() ? FieldState::Unavailable : FieldState::ProviderSupplied};
    const String emergency = emergencyFromSquawk(squawk);
    ac.emergency = {emergency, emergency.isEmpty() ? FieldState::Unavailable : FieldState::ProviderSupplied};
    ac.sourceType = {"opensky", FieldState::ProviderSupplied};
    if (!row[4].isNull()) ac.lastReceivedMs = {static_cast<uint64_t>(row[4].as<uint64_t>()) * 1000ULL, FieldState::ProviderSupplied};
    geo::Location pos {ac.latitude.value, ac.longitude.value};
    ac.distanceNm = geo::distanceNm(origin, pos);
    if (ac.distanceNm > radiusNm) continue;
    ac.bearingDeg = geo::bearingDeg(origin, pos);
    ac.provider = ProviderId::OpenSky;
    ac.stale = false;
    out->push_back(ac);
  }
  return true;
}

bool OpenSkyProvider::fetchOptionalRoute(Aircraft*) { return false; }

ProviderCapabilities OpenSkyProvider::getCapabilities() const {
  ProviderCapabilities caps;
  caps.minPollIntervalSec = 10;
  caps.attribution = "OpenSky Network";
  return caps;
}

ProviderStatus OpenSkyProvider::getProviderStatus() const { return status_; }
void OpenSkyProvider::cancelPendingRequest() {}

ProviderError OpenSkyProvider::mapError(int httpCode, const String&) const {
  if (httpCode == 401 || httpCode == 403) return ProviderError::Auth;
  if (httpCode == 429) return ProviderError::RateLimited;
  if (httpCode <= 0) return ProviderError::Timeout;
  return ProviderError::Http;
}

void OpenSkyProvider::shutdown() {}

}  // namespace micro_radar
