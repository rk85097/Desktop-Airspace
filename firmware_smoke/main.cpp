#include <Arduino.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_system.h"

#include "../firmware/include/display/SpotpearSmokeDisplay.h"
#include "AircraftSprite.inc"

#include <string.h>

namespace {
constexpr int kWidth = 480;
constexpr int kHeight = 480;
constexpr int kScopeRadius = 236;
constexpr int kScopeEdgeRadius = 235;
constexpr int kProjectionRadius = 232;
constexpr const char* kRendererRevision = "layered-r13-ui-polish";

constexpr gpio_num_t kSpiCs = GPIO_NUM_42;
constexpr gpio_num_t kSpiSck = GPIO_NUM_2;
constexpr gpio_num_t kSpiSda = GPIO_NUM_1;
constexpr gpio_num_t kBacklight = GPIO_NUM_6;
constexpr ledc_channel_t kBacklightChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_t kBacklightTimer = LEDC_TIMER_0;

esp_lcd_panel_handle_t panel = nullptr;
uint16_t* framebuffer = nullptr;
uint16_t* static_framebuffer = nullptr;
uint16_t* flush_stripe = nullptr;
uint16_t* dirty_framebuffer = nullptr;
float sweep_angle = 0.0f;
uint32_t last_sweep_ms = 0;
MicroRadarDisplayState display_state = {};
bool output_rotation_enabled = true;
uint8_t applied_brightness = 0;
uint32_t frame_counter = 0;
uint32_t fps_window_ms = 0;
uint16_t measured_fps = 0;
bool static_cache_valid = false;
bool static_cache_available = false;
uint32_t static_cache_hits = 0;
uint32_t static_cache_misses = 0;
uint32_t static_cache_hash = 0;
bool detail_cache_valid = false;
uint32_t detail_cache_hash = 0;
float previous_sweep_angle = 0.0f;
bool previous_sweep_valid = false;
uint32_t partial_flushes = 0;
uint32_t full_flushes = 0;
uint32_t packed_flushes = 0;
uint32_t stripe_flushes = 0;
constexpr int kDirtyBufferMaxPixels = 260 * 260;

struct LcdCommand {
  uint8_t command;
  const uint8_t* data;
  uint8_t data_size;
  uint16_t delay_ms;
};

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t rgb888To565(uint32_t rgb) {
  return rgb565((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
}

uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t alpha) {
  const uint8_t inv = 255 - alpha;
  const uint8_t br = ((bg >> 11) & 0x1F) << 3;
  const uint8_t bgc = ((bg >> 5) & 0x3F) << 2;
  const uint8_t bb = (bg & 0x1F) << 3;
  const uint8_t fr = ((fg >> 11) & 0x1F) << 3;
  const uint8_t fgc = ((fg >> 5) & 0x3F) << 2;
  const uint8_t fb = (fg & 0x1F) << 3;
  return rgb565((fr * alpha + br * inv) / 255, (fgc * alpha + bgc * inv) / 255, (fb * alpha + bb * inv) / 255);
}

uint16_t boost565(uint16_t color, uint8_t percent) {
  const uint8_t r = min(255, ((((color >> 11) & 0x1F) << 3) * percent) / 100);
  const uint8_t g = min(255, ((((color >> 5) & 0x3F) << 2) * percent) / 100);
  const uint8_t b = min(255, (((color & 0x1F) << 3) * percent) / 100);
  return rgb565(r, g, b);
}

uint16_t tint565PreserveLight(uint16_t src, uint16_t tint) {
  const uint8_t sr = ((src >> 11) & 0x1F) << 3;
  const uint8_t sg = ((src >> 5) & 0x3F) << 2;
  const uint8_t sb = (src & 0x1F) << 3;
  const uint8_t tr = ((tint >> 11) & 0x1F) << 3;
  const uint8_t tg = ((tint >> 5) & 0x3F) << 2;
  const uint8_t tb = (tint & 0x1F) << 3;
  const uint16_t luma = (static_cast<uint16_t>(sr) * 77 + static_cast<uint16_t>(sg) * 150 + static_cast<uint16_t>(sb) * 29) >> 8;
  const uint16_t shade = max<uint16_t>(54, min<uint16_t>(255, luma + 28));
  uint8_t r = min<uint16_t>(255, (static_cast<uint16_t>(tr) * shade) / 190);
  uint8_t g = min<uint16_t>(255, (static_cast<uint16_t>(tg) * shade) / 190);
  uint8_t b = min<uint16_t>(255, (static_cast<uint16_t>(tb) * shade) / 190);
  if (luma > 210) {
    const uint8_t hi = min<uint16_t>(150, (luma - 210) * 3);
    r = (r * (255 - hi) + 255 * hi) / 255;
    g = (g * (255 - hi) + 255 * hi) / 255;
    b = (b * (255 - hi) + 255 * hi) / 255;
  }
  return rgb565(r, g, b);
}

uint32_t hashByte(uint32_t h, uint8_t value) {
  h ^= value;
  return h * 16777619UL;
}

uint32_t hashBytes(uint32_t h, const void* data, size_t len) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < len; ++i) h = hashByte(h, bytes[i]);
  return h;
}

uint32_t staticLayerHash() {
  uint32_t h = 2166136261UL;
  const uint16_t mapCount = display_state.mapSegmentCount > MICRO_RADAR_MAX_DISPLAY_MAP_SEGMENTS ? MICRO_RADAR_MAX_DISPLAY_MAP_SEGMENTS : display_state.mapSegmentCount;
  const uint8_t pointCount = display_state.mapPointCount > MICRO_RADAR_MAX_DISPLAY_MAP_POINTS ? MICRO_RADAR_MAX_DISPLAY_MAP_POINTS : display_state.mapPointCount;
  h = hashBytes(h, &display_state.rangeNm, sizeof(display_state.rangeNm));
  h = hashBytes(h, &display_state.aircraftRendered, sizeof(display_state.aircraftRendered));
  h = hashBytes(h, &display_state.mapEnabled, sizeof(display_state.mapEnabled));
  h = hashBytes(h, &display_state.airportsEnabled, sizeof(display_state.airportsEnabled));
  h = hashBytes(h, &display_state.airportLabelsEnabled, sizeof(display_state.airportLabelsEnabled));
  h = hashBytes(h, &display_state.rangeRingsEnabled, sizeof(display_state.rangeRingsEnabled));
  h = hashBytes(h, &display_state.outerRangeRingEnabled, sizeof(display_state.outerRangeRingEnabled));
  h = hashBytes(h, &display_state.scopeEdgeEnabled, sizeof(display_state.scopeEdgeEnabled));
  h = hashBytes(h, &display_state.cardinalLabelsEnabled, sizeof(display_state.cardinalLabelsEnabled));
  h = hashBytes(h, &display_state.ordinalLabelsEnabled, sizeof(display_state.ordinalLabelsEnabled));
  h = hashBytes(h, &display_state.rangeRingLabelsEnabled, sizeof(display_state.rangeRingLabelsEnabled));
  h = hashBytes(h, &display_state.crosshairEnabled, sizeof(display_state.crosshairEnabled));
  h = hashBytes(h, &display_state.metricUnits, sizeof(display_state.metricUnits));
  h = hashBytes(h, &display_state.lightTheme, sizeof(display_state.lightTheme));
  h = hashBytes(h, &display_state.fontStyle, sizeof(display_state.fontStyle));
  h = hashBytes(h, &display_state.displayRotation, sizeof(display_state.displayRotation));
  h = hashBytes(h, &display_state.airportIconType, sizeof(display_state.airportIconType));
  h = hashBytes(h, &display_state.airportLabelScale, sizeof(display_state.airportLabelScale));
  h = hashBytes(h, &display_state.aircraftLabelSpacing, sizeof(display_state.aircraftLabelSpacing));
  h = hashBytes(h, &display_state.rangeRingStyle, sizeof(display_state.rangeRingStyle));
  h = hashBytes(h, &display_state.rangeRingThickness, sizeof(display_state.rangeRingThickness));
  h = hashBytes(h, &display_state.scopeEdgeThickness, sizeof(display_state.scopeEdgeThickness));
  h = hashBytes(h, &display_state.crosshairStyle, sizeof(display_state.crosshairStyle));
  h = hashBytes(h, &display_state.crosshairThickness, sizeof(display_state.crosshairThickness));
  h = hashBytes(h, &display_state.landColor, sizeof(display_state.landColor));
  h = hashBytes(h, &display_state.waterColor, sizeof(display_state.waterColor));
  h = hashBytes(h, &display_state.airportColor, sizeof(display_state.airportColor));
  h = hashBytes(h, &display_state.airportLabelColor, sizeof(display_state.airportLabelColor));
  h = hashBytes(h, &display_state.scopeBackgroundColor, sizeof(display_state.scopeBackgroundColor));
  h = hashBytes(h, &display_state.scopeOutsideColor, sizeof(display_state.scopeOutsideColor));
  h = hashBytes(h, &display_state.mapCoastColor, sizeof(display_state.mapCoastColor));
  h = hashBytes(h, &display_state.mapBorderColor, sizeof(display_state.mapBorderColor));
  h = hashBytes(h, &display_state.mapWaterLineColor, sizeof(display_state.mapWaterLineColor));
  h = hashBytes(h, &display_state.rasterBackgroundReady, sizeof(display_state.rasterBackgroundReady));
  h = hashBytes(h, &display_state.rasterBackgroundHash, sizeof(display_state.rasterBackgroundHash));
  h = hashBytes(h, &display_state.mapBrightness, sizeof(display_state.mapBrightness));
  h = hashBytes(h, &display_state.rangeRingColor, sizeof(display_state.rangeRingColor));
  h = hashBytes(h, &display_state.crosshairColor, sizeof(display_state.crosshairColor));
  h = hashBytes(h, &display_state.labelColor, sizeof(display_state.labelColor));
  h = hashBytes(h, &display_state.cardinalLabelColor, sizeof(display_state.cardinalLabelColor));
  h = hashBytes(h, &display_state.ordinalLabelColor, sizeof(display_state.ordinalLabelColor));
  h = hashBytes(h, &mapCount, sizeof(mapCount));
  h = hashBytes(h, display_state.mapSegments, sizeof(MicroRadarDisplaySegment) * mapCount);
  h = hashBytes(h, &pointCount, sizeof(pointCount));
  h = hashBytes(h, display_state.mapPoints, sizeof(MicroRadarDisplayMapPoint) * pointCount);
  return h;
}

uint32_t detailLayerHash() {
  uint32_t h = 2166136261UL;
  h = hashBytes(h, &display_state.detailOpen, sizeof(display_state.detailOpen));
  h = hashBytes(h, &display_state.fontStyle, sizeof(display_state.fontStyle));
  h = hashBytes(h, &display_state.brightness, sizeof(display_state.brightness));
  h = hashBytes(h, &display_state.scopeOutsideColor, sizeof(display_state.scopeOutsideColor));
  h = hashBytes(h, &display_state.waterColor, sizeof(display_state.waterColor));
  h = hashBytes(h, &display_state.labelColor, sizeof(display_state.labelColor));
  h = hashBytes(h, &display_state.detailLabelColor, sizeof(display_state.detailLabelColor));
  h = hashBytes(h, &display_state.detailBackgroundColor, sizeof(display_state.detailBackgroundColor));
  h = hashBytes(h, &display_state.aircraftRendered, sizeof(display_state.aircraftRendered));
  const uint16_t rendered = display_state.aircraftRendered > MICRO_RADAR_MAX_DISPLAY_AIRCRAFT ? MICRO_RADAR_MAX_DISPLAY_AIRCRAFT : display_state.aircraftRendered;
  for (uint16_t i = 0; i < rendered; ++i) {
    if (!display_state.aircraft[i].selected) continue;
    h = hashBytes(h, &display_state.aircraft[i], sizeof(display_state.aircraft[i]));
    return h;
  }
  return h;
}

struct DirtyRect {
  int x0 {kWidth};
  int y0 {kHeight};
  int x1 {0};
  int y1 {0};
  bool valid {false};
};

DirtyRect previous_dynamic_rect;
bool previous_dynamic_rect_valid = false;

void includeRect(DirtyRect* r, int x0, int y0, int x1, int y1) {
  if (!r) return;
  x0 = max(0, min(kWidth, x0));
  y0 = max(0, min(kHeight, y0));
  x1 = max(0, min(kWidth, x1));
  y1 = max(0, min(kHeight, y1));
  if (x1 <= x0 || y1 <= y0) return;
  if (!r->valid) {
    r->x0 = x0;
    r->y0 = y0;
    r->x1 = x1;
    r->y1 = y1;
    r->valid = true;
    return;
  }
  r->x0 = min(r->x0, x0);
  r->y0 = min(r->y0, y0);
  r->x1 = max(r->x1, x1);
  r->y1 = max(r->y1, y1);
}

void includeSweepRect(DirtyRect* r, float angleDeg, uint8_t widthDeg) {
  const uint8_t width = max<uint8_t>(6, min<uint8_t>(90, widthDeg));
  int minX = 240;
  int minY = 240;
  int maxX = 240;
  int maxY = 240;
  for (uint8_t i = 0; i <= 8; ++i) {
    const float sampleDeg = angleDeg - (static_cast<float>(width) * static_cast<float>(i) / 8.0f);
    const float a = sampleDeg * 0.01745329252f;
    const int x2 = 240 + static_cast<int>(cosf(a) * kProjectionRadius);
    const int y2 = 240 + static_cast<int>(sinf(a) * kProjectionRadius);
    minX = min(minX, x2);
    minY = min(minY, y2);
    maxX = max(maxX, x2);
    maxY = max(maxY, y2);
  }
  const int pad = 18;
  includeRect(r, minX - pad, minY - pad, maxX + pad + 1, maxY + pad + 1);
}

void includeDynamicRect(DirtyRect* r, bool includeAircraft) {
  includeRect(r, 112, 82, 368, 126);
  if (display_state.sweepLineEnabled) {
    const uint8_t width = display_state.sweepFadeWidthDeg ? display_state.sweepFadeWidthDeg : 24;
    includeSweepRect(r, sweep_angle, width);
    if (previous_sweep_valid) includeSweepRect(r, previous_sweep_angle, width);
  }
  if (!includeAircraft) return;
  const uint16_t rendered = display_state.aircraftRendered > MICRO_RADAR_MAX_DISPLAY_AIRCRAFT ? MICRO_RADAR_MAX_DISPLAY_AIRCRAFT : display_state.aircraftRendered;
  for (uint16_t i = 0; i < rendered; ++i) {
    const int cx = 240 + display_state.aircraft[i].x;
    const int cy = 240 + display_state.aircraft[i].y;
    const int textScale = max(70, min(220, display_state.aircraftTextScale ? static_cast<int>(display_state.aircraftTextScale) : 100));
    const int labelHeight = max(28, (36 * textScale + 50) / 100);
    includeRect(r, cx - 70, cy - 70, cx + 225, cy + 70 + labelHeight);
    if (display_state.trailsEnabled) {
      const uint8_t trailCount = display_state.aircraft[i].trailCount > 8 ? 8 : display_state.aircraft[i].trailCount;
      for (uint8_t j = 0; j < trailCount; ++j) {
        const int tx = 240 + display_state.aircraft[i].trailX[j];
        const int ty = 240 + display_state.aircraft[i].trailY[j];
        includeRect(r, tx - 8, ty - 8, tx + 9, ty + 9);
      }
    }
  }
}

void flushRect(const DirtyRect& rect) {
  if (!rect.valid) {
    esp_lcd_panel_draw_bitmap(panel, 0, 0, kWidth, kHeight, framebuffer);
    ++full_flushes;
    return;
  }
  const int x0 = max(0, min(kWidth, rect.x0));
  const int y0 = max(0, min(kHeight, rect.y0));
  const int x1 = max(0, min(kWidth, rect.x1));
  const int y1 = max(0, min(kHeight, rect.y1));
  const int w = x1 - x0;
  if (w <= 0 || y1 <= y0) return;
  const int h = y1 - y0;
  const size_t pixels = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (dirty_framebuffer && pixels <= kDirtyBufferMaxPixels) {
    for (int row = 0; row < h; ++row) {
      memcpy(dirty_framebuffer + row * w, framebuffer + (y0 + row) * kWidth + x0, w * sizeof(uint16_t));
    }
    esp_lcd_panel_draw_bitmap(panel, x0, y0, x1, y1, dirty_framebuffer);
    ++partial_flushes;
    ++packed_flushes;
    return;
  }
  if (!flush_stripe) {
    esp_lcd_panel_draw_bitmap(panel, 0, 0, kWidth, kHeight, framebuffer);
    ++full_flushes;
    return;
  }
  constexpr int kStripeRows = 12;
  for (int y = y0; y < y1; y += kStripeRows) {
    const int rows = min(kStripeRows, y1 - y);
    for (int row = 0; row < rows; ++row) {
      memcpy(flush_stripe + row * w, framebuffer + (y + row) * kWidth + x0, w * sizeof(uint16_t));
    }
    esp_lcd_panel_draw_bitmap(panel, x0, y, x1, y + rows, flush_stripe);
  }
  ++partial_flushes;
  ++stripe_flushes;
}

void restoreRectFromStatic(const DirtyRect& rect) {
  if (!static_framebuffer || !rect.valid) return;
  const int x0 = max(0, min(kWidth, rect.x0));
  const int y0 = max(0, min(kHeight, rect.y0));
  const int x1 = max(0, min(kWidth, rect.x1));
  const int y1 = max(0, min(kHeight, rect.y1));
  const int w = x1 - x0;
  if (w <= 0 || y1 <= y0) return;
  for (int row = y0; row < y1; ++row) {
    memcpy(framebuffer + row * kWidth + x0, static_framebuffer + row * kWidth + x0, w * sizeof(uint16_t));
  }
}

void initBacklight() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = kBacklightTimer;
  timer.freq_hz = 20000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {};
  channel.gpio_num = kBacklight;
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = kBacklightChannel;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.timer_sel = kBacklightTimer;
  channel.duty = 255;
  channel.hpoint = 0;
  ledc_channel_config(&channel);
  applied_brightness = 255;
}

void applyBacklightBrightness(uint8_t brightness) {
  const uint8_t duty = brightness == 0 ? 1 : brightness;
  if (duty == applied_brightness) return;
  applied_brightness = duty;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, kBacklightChannel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, kBacklightChannel);
}

uint16_t scale565(uint16_t color, uint8_t brightness) {
  if (brightness >= 250) return color;
  const uint8_t r = ((color >> 11) & 0x1F) << 3;
  const uint8_t g = ((color >> 5) & 0x3F) << 2;
  const uint8_t b = (color & 0x1F) << 3;
  return rgb565((r * brightness) / 255, (g * brightness) / 255, (b * brightness) / 255);
}

void applySoftwareBrightnessFallback(uint8_t brightness) {
  if (brightness >= 245) return;
  const uint32_t count = kWidth * kHeight;
  for (uint32_t i = 0; i < count; ++i) {
    framebuffer[i] = scale565(framebuffer[i], brightness);
  }
}

void write9(bool data_bit, uint8_t value) {
  gpio_set_level(kSpiCs, 0);
  gpio_set_level(kSpiSck, 0);

  gpio_set_level(kSpiSda, data_bit ? 1 : 0);
  gpio_set_level(kSpiSck, 1);
  gpio_set_level(kSpiSck, 0);

  for (int bit = 7; bit >= 0; --bit) {
    gpio_set_level(kSpiSda, (value >> bit) & 0x01);
    gpio_set_level(kSpiSck, 1);
    gpio_set_level(kSpiSck, 0);
  }

  gpio_set_level(kSpiCs, 1);
}

void sendCommand(uint8_t command) {
  write9(false, command);
}

void sendData(uint8_t data) {
  write9(true, data);
}

void sendCommandData(uint8_t command, const uint8_t* data, uint8_t data_size, uint16_t delay_ms) {
  sendCommand(command);
  for (uint8_t i = 0; i < data_size; ++i) {
    sendData(data[i]);
  }
  if (delay_ms > 0) {
    delay(delay_ms);
  }
}

void init3WireSpi() {
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << kSpiCs) | (1ULL << kSpiSck) | (1ULL << kSpiSda);
  gpio_config(&io_conf);
  gpio_set_level(kSpiCs, 1);
  gpio_set_level(kSpiSck, 0);
  gpio_set_level(kSpiSda, 0);
}

void initSt7701() {
  static const uint8_t c_ff_13[] = {0x77, 0x01, 0x00, 0x00, 0x13};
  static const uint8_t c_ef_08[] = {0x08};
  static const uint8_t c_ff_10[] = {0x77, 0x01, 0x00, 0x00, 0x10};
  static const uint8_t c_c0[] = {0x3B, 0x00};
  static const uint8_t c_c1[] = {0x10, 0x0C};
  static const uint8_t c_c2[] = {0x07, 0x0A};
  static const uint8_t c_c7[] = {0x00};
  static const uint8_t c_cc[] = {0x10};
  static const uint8_t c_cd[] = {0x08};
  static const uint8_t c_b0[] = {0x05, 0x12, 0x98, 0x0E, 0x0F, 0x07, 0x07, 0x09, 0x09, 0x23, 0x05, 0x52, 0x0F, 0x67, 0x2C, 0x11};
  static const uint8_t c_b1[] = {0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08, 0x08, 0x22, 0x03, 0x51, 0x11, 0x66, 0x2B, 0x0F};
  static const uint8_t c_ff_11[] = {0x77, 0x01, 0x00, 0x00, 0x11};
  static const uint8_t c_b0_11[] = {0x5D};
  static const uint8_t c_b1_11[] = {0x3E};
  static const uint8_t c_b2_11[] = {0x81};
  static const uint8_t c_b3_11[] = {0x80};
  static const uint8_t c_b5_11[] = {0x4E};
  static const uint8_t c_b7_11[] = {0x85};
  static const uint8_t c_b8_11[] = {0x20};
  static const uint8_t c_c1_11[] = {0x78};
  static const uint8_t c_c2_11[] = {0x78};
  static const uint8_t c_d0_11[] = {0x88};
  static const uint8_t c_e0[] = {0x00, 0x00, 0x02};
  static const uint8_t c_e1[] = {0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30, 0x00, 0x33, 0x33};
  static const uint8_t c_e2[] = {0x11, 0x11, 0x33, 0x33, 0xF4, 0x00, 0x00, 0x00, 0xF4, 0x00, 0x00, 0x00};
  static const uint8_t c_e3[] = {0x00, 0x00, 0x11, 0x11};
  static const uint8_t c_e4[] = {0x44, 0x44};
  static const uint8_t c_e5[] = {0x0D, 0xF5, 0x30, 0xF0, 0x0F, 0xF7, 0x30, 0xF0, 0x09, 0xF1, 0x30, 0xF0, 0x0B, 0xF3, 0x30, 0xF0};
  static const uint8_t c_e6[] = {0x00, 0x00, 0x11, 0x11};
  static const uint8_t c_e7[] = {0x44, 0x44};
  static const uint8_t c_e8[] = {0x0C, 0xF4, 0x30, 0xF0, 0x0E, 0xF6, 0x30, 0xF0, 0x08, 0xF0, 0x30, 0xF0, 0x0A, 0xF2, 0x30, 0xF0};
  static const uint8_t c_e9[] = {0x36, 0x01};
  static const uint8_t c_eb[] = {0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40};
  static const uint8_t c_ed[] = {0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF, 0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF};
  static const uint8_t c_ef[] = {0x08, 0x08, 0x08, 0x45, 0x3F, 0x54};
  static const uint8_t c_ff_00[] = {0x77, 0x01, 0x00, 0x00, 0x00};
  static const uint8_t c_3a[] = {0x66};
  static const uint8_t c_36[] = {0x00};
  static const uint8_t c_00[] = {0x00};

  const LcdCommand commands[] = {
      {0xFF, c_ff_13, sizeof(c_ff_13), 0}, {0xEF, c_ef_08, sizeof(c_ef_08), 0},
      {0xFF, c_ff_10, sizeof(c_ff_10), 0}, {0xC0, c_c0, sizeof(c_c0), 0},
      {0xC1, c_c1, sizeof(c_c1), 0},       {0xC2, c_c2, sizeof(c_c2), 0},
      {0xC7, c_c7, sizeof(c_c7), 0},       {0xCC, c_cc, sizeof(c_cc), 0},
      {0xCD, c_cd, sizeof(c_cd), 0},       {0xB0, c_b0, sizeof(c_b0), 0},
      {0xB1, c_b1, sizeof(c_b1), 0},       {0xFF, c_ff_11, sizeof(c_ff_11), 0},
      {0xB0, c_b0_11, sizeof(c_b0_11), 0}, {0xB1, c_b1_11, sizeof(c_b1_11), 0},
      {0xB2, c_b2_11, sizeof(c_b2_11), 0}, {0xB3, c_b3_11, sizeof(c_b3_11), 0},
      {0xB5, c_b5_11, sizeof(c_b5_11), 0}, {0xB7, c_b7_11, sizeof(c_b7_11), 0},
      {0xB8, c_b8_11, sizeof(c_b8_11), 0}, {0xC1, c_c1_11, sizeof(c_c1_11), 0},
      {0xC2, c_c2_11, sizeof(c_c2_11), 0}, {0xD0, c_d0_11, sizeof(c_d0_11), 0},
      {0xE0, c_e0, sizeof(c_e0), 0},       {0xE1, c_e1, sizeof(c_e1), 0},
      {0xE2, c_e2, sizeof(c_e2), 0},       {0xE3, c_e3, sizeof(c_e3), 0},
      {0xE4, c_e4, sizeof(c_e4), 0},       {0xE5, c_e5, sizeof(c_e5), 0},
      {0xE6, c_e6, sizeof(c_e6), 0},       {0xE7, c_e7, sizeof(c_e7), 0},
      {0xE8, c_e8, sizeof(c_e8), 0},       {0xE9, c_e9, sizeof(c_e9), 0},
      {0xEB, c_eb, sizeof(c_eb), 0},       {0xED, c_ed, sizeof(c_ed), 0},
      {0xEF, c_ef, sizeof(c_ef), 0},       {0xFF, c_ff_00, sizeof(c_ff_00), 0},
      {0x11, c_00, 0, 120},                {0x3A, c_3a, sizeof(c_3a), 0},
      {0x36, c_36, sizeof(c_36), 0},       {0x35, c_00, 0, 0},
      {0x20, c_00, 0, 120},                {0x29, c_00, 0, 0},
  };

  for (const auto& cmd : commands) {
    sendCommandData(cmd.command, cmd.data, cmd.data_size, cmd.delay_ms);
  }
}

bool initRgbPanel() {
  esp_lcd_rgb_panel_config_t config = {};
  config.clk_src = LCD_CLK_SRC_PLL160M;
  config.timings.pclk_hz = 12 * 1000 * 1000;
  config.timings.h_res = kWidth;
  config.timings.v_res = kHeight;
  config.timings.hsync_pulse_width = 8;
  config.timings.hsync_back_porch = 10;
  config.timings.hsync_front_porch = 50;
  config.timings.vsync_pulse_width = 2;
  config.timings.vsync_back_porch = 18;
  config.timings.vsync_front_porch = 8;
  config.timings.flags.pclk_active_neg = 0;
  config.data_width = 16;
  config.sram_trans_align = 4;
  config.psram_trans_align = 64;
  config.hsync_gpio_num = 38;
  config.vsync_gpio_num = 39;
  config.de_gpio_num = 40;
  config.pclk_gpio_num = 41;
  config.disp_gpio_num = -1;
  const int pins[16] = {5, 45, 48, 47, 21, 14, 13, 12, 11, 10, 9, 46, 3, 8, 18, 17};
  for (int i = 0; i < 16; ++i) {
    config.data_gpio_nums[i] = pins[i];
  }
  config.flags.fb_in_psram = 1;

  esp_err_t err = esp_lcd_new_rgb_panel(&config, &panel);
  Serial.printf("esp_lcd_new_rgb_panel: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return false;
  err = esp_lcd_panel_reset(panel);
  Serial.printf("esp_lcd_panel_reset: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return false;
  err = esp_lcd_panel_init(panel);
  Serial.printf("esp_lcd_panel_init: %s\n", esp_err_to_name(err));
  return err == ESP_OK;
}

void putPixel(int x, int y, uint16_t color) {
  if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
  const int ox = x;
  const int oy = y;
  if (output_rotation_enabled) {
    switch (display_state.displayRotation & 0x03) {
      case 1:
        x = kWidth - 1 - oy;
        y = ox;
        break;
      case 2:
        x = kWidth - 1 - ox;
        y = kHeight - 1 - oy;
        break;
      case 3:
        x = oy;
        y = kHeight - 1 - ox;
        break;
      default:
        break;
    }
  }
  framebuffer[y * kWidth + x] = color;
}

void blendPixel(int x, int y, uint16_t color, uint8_t alpha) {
  if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
  const int ox = x;
  const int oy = y;
  if (output_rotation_enabled) {
    switch (display_state.displayRotation & 0x03) {
      case 1:
        x = kWidth - 1 - oy;
        y = ox;
        break;
      case 2:
        x = kWidth - 1 - ox;
        y = kHeight - 1 - oy;
        break;
      case 3:
        x = oy;
        y = kHeight - 1 - ox;
        break;
      default:
        break;
    }
  }
  uint16_t& dst = framebuffer[y * kWidth + x];
  dst = blend565(dst, color, alpha);
}

void drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    putPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void drawLineAlpha(int x0, int y0, int x1, int y1, uint16_t color, uint8_t alpha) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    blendPixel(x0, y0, color, alpha);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void drawWideLineAlpha(int x0, int y0, int x1, int y1, uint16_t color, uint8_t alpha, uint8_t width) {
  const int dx = x1 - x0;
  const int dy = y1 - y0;
  const float len = sqrtf(static_cast<float>(dx * dx + dy * dy));
  if (len < 1.0f) {
    blendPixel(x0, y0, color, alpha);
    return;
  }
  const float nx = -dy / len;
  const float ny = dx / len;
  const int half = width / 2;
  for (int o = -half; o <= half; ++o) {
    const uint8_t lineAlpha = o == 0 ? alpha : static_cast<uint8_t>(alpha / (abs(o) + 1));
    drawLineAlpha(x0 + static_cast<int>(roundf(nx * o)), y0 + static_cast<int>(roundf(ny * o)),
                  x1 + static_cast<int>(roundf(nx * o)), y1 + static_cast<int>(roundf(ny * o)), color, lineAlpha);
  }
}

void drawDashedLineAlpha(int x0, int y0, int x1, int y1, uint16_t color, uint8_t alpha, uint8_t dash = 7, uint8_t gap = 5) {
  const int dx = x1 - x0;
  const int dy = y1 - y0;
  const float len = sqrtf(static_cast<float>(dx * dx + dy * dy));
  if (len < 1.0f) return;
  const float ux = dx / len;
  const float uy = dy / len;
  for (float d = 0; d < len; d += dash + gap) {
    const float e = min(len, d + dash);
    drawLineAlpha(x0 + static_cast<int>(roundf(ux * d)), y0 + static_cast<int>(roundf(uy * d)),
                  x0 + static_cast<int>(roundf(ux * e)), y0 + static_cast<int>(roundf(uy * e)), color, alpha);
  }
}

void markCoastRows(int x0, int y0, int x1, int y1, int* coastXByRow) {
  if (abs(y1 - y0) < 2) return;
  if (y0 > y1) {
    const int tx = x0; x0 = x1; x1 = tx;
    const int ty = y0; y0 = y1; y1 = ty;
  }
  y0 = max(12, min(468, y0));
  y1 = max(12, min(468, y1));
  for (int y = y0; y <= y1; ++y) {
    const float t = static_cast<float>(y - y0) / static_cast<float>(max(1, y1 - y0));
    const int x = x0 + static_cast<int>((x1 - x0) * t);
    const int coastX = max(12, min(468, x));
    if (coastXByRow[y] < 0 || coastX < coastXByRow[y]) coastXByRow[y] = coastX;
  }
}

void fillLandFromCoastRows(const int* coastXByRow, uint16_t land) {
  int lastCoastX = -1;
  for (int y = 12; y <= 468; ++y) {
    int coastX = coastXByRow[y];
    if (coastX < 0) coastX = lastCoastX;
    if (coastX < 0) continue;
    lastCoastX = coastX;
    for (int xx = coastX; xx < 468; ++xx) {
      const int dx = xx - 240;
      const int dy = y - 240;
      if (dx * dx + dy * dy <= kScopeRadius * kScopeRadius) putPixel(xx, y, land);
    }
  }
}

void drawCircle(int cx, int cy, int r, uint16_t color) {
  int x = -r, y = 0, err = 2 - 2 * r;
  do {
    putPixel(cx - x, cy + y, color);
    putPixel(cx - y, cy - x, color);
    putPixel(cx + x, cy - y, color);
    putPixel(cx + y, cy + x, color);
    int e2 = err;
    if (e2 <= y) err += ++y * 2 + 1;
    if (e2 > x || err > y) err += ++x * 2 + 1;
  } while (x < 0);
}

void drawCircleAlpha(int cx, int cy, int r, uint16_t color, uint8_t alpha) {
  int x = -r, y = 0, err = 2 - 2 * r;
  do {
    blendPixel(cx - x, cy + y, color, alpha);
    blendPixel(cx - y, cy - x, color, alpha);
    blendPixel(cx + x, cy - y, color, alpha);
    blendPixel(cx + y, cy + x, color, alpha);
    int e2 = err;
    if (e2 <= y) err += ++y * 2 + 1;
    if (e2 > x || err > y) err += ++x * 2 + 1;
  } while (x < 0);
}

void drawThickCircleAlpha(int cx, int cy, int r, uint16_t color, uint8_t alpha, uint8_t thickness) {
  const int half = max(0, static_cast<int>(thickness) / 2);
  for (int o = -half; o <= half; ++o) {
    if (r + o > 0) drawCircleAlpha(cx, cy, r + o, color, alpha);
  }
}

void drawDashedCircleAlpha(int cx, int cy, int r, uint16_t color, uint8_t alpha, uint8_t thickness) {
  constexpr float degToRad = 0.01745329252f;
  for (int deg = 0; deg < 360; deg += 18) {
    const float a0 = deg * degToRad;
    const float a1 = (deg + 10) * degToRad;
    drawWideLineAlpha(cx + static_cast<int>(cosf(a0) * r), cy + static_cast<int>(sinf(a0) * r),
                      cx + static_cast<int>(cosf(a1) * r), cy + static_cast<int>(sinf(a1) * r), color, alpha, thickness);
  }
}

void fillRect(int x, int y, int w, int h, uint16_t color) {
  for (int yy = y; yy < y + h; ++yy) {
    for (int xx = x; xx < x + w; ++xx) {
      putPixel(xx, yy, color);
    }
  }
}

void fillRectAlpha(int x, int y, int w, int h, uint16_t color, uint8_t alpha) {
  for (int yy = y; yy < y + h; ++yy) {
    for (int xx = x; xx < x + w; ++xx) {
      blendPixel(xx, yy, color, alpha);
    }
  }
}

void fillRoundRectAlpha(int x, int y, int w, int h, int r, uint16_t color, uint8_t alpha) {
  fillRectAlpha(x + r, y, w - 2 * r, h, color, alpha);
  fillRectAlpha(x, y + r, w, h - 2 * r, color, alpha);
  for (int yy = -r; yy <= r; ++yy) {
    for (int xx = -r; xx <= r; ++xx) {
      if (xx * xx + yy * yy <= r * r) {
        blendPixel(x + r + xx, y + r + yy, color, alpha);
        blendPixel(x + w - r - 1 + xx, y + r + yy, color, alpha);
        blendPixel(x + r + xx, y + h - r - 1 + yy, color, alpha);
        blendPixel(x + w - r - 1 + xx, y + h - r - 1 + yy, color, alpha);
      }
    }
  }
}

void drawFilledCircle(int cx, int cy, int r, uint16_t color) {
  for (int y = -r; y <= r; ++y) {
    for (int x = -r; x <= r; ++x) {
      if (x * x + y * y <= r * r) putPixel(cx + x, cy + y, color);
    }
  }
}

void drawFilledCircleAlpha(int cx, int cy, int r, uint16_t color, uint8_t alpha) {
  for (int y = -r; y <= r; ++y) {
    for (int x = -r; x <= r; ++x) {
      if (x * x + y * y <= r * r) blendPixel(cx + x, cy + y, color, alpha);
    }
  }
}

int edge(int ax, int ay, int bx, int by, int px, int py) {
  return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

void drawFilledTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
  const int minX = max(0, min(x0, min(x1, x2)));
  const int maxX = min(kWidth - 1, max(x0, max(x1, x2)));
  const int minY = max(0, min(y0, min(y1, y2)));
  const int maxY = min(kHeight - 1, max(y0, max(y1, y2)));
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const int e0 = edge(x0, y0, x1, y1, x, y);
      const int e1 = edge(x1, y1, x2, y2, x, y);
      const int e2 = edge(x2, y2, x0, y0, x, y);
      if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) putPixel(x, y, color);
    }
  }
}

void drawFilledTriangleAlpha(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color, uint8_t alpha) {
  const int minX = max(0, min(x0, min(x1, x2)));
  const int maxX = min(kWidth - 1, max(x0, max(x1, x2)));
  const int minY = max(0, min(y0, min(y1, y2)));
  const int maxY = min(kHeight - 1, max(y0, max(y1, y2)));
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const int e0 = edge(x0, y0, x1, y1, x, y);
      const int e1 = edge(x1, y1, x2, y2, x, y);
      const int e2 = edge(x2, y2, x0, y0, x, y);
      if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) blendPixel(x, y, color, alpha);
    }
  }
}

void drawFilledQuadAlpha(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color, uint8_t alpha) {
  drawFilledTriangleAlpha(x0, y0, x1, y1, x2, y2, color, alpha);
  drawFilledTriangleAlpha(x0, y0, x2, y2, x3, y3, color, alpha);
}

void drawSweepWedge(float angle, uint16_t color, uint8_t widthDeg) {
  const int cx = 240;
  const int cy = 240;
  const uint8_t width = max(8, min(90, static_cast<int>(widthDeg)));
  const int steps = max(6, min(18, static_cast<int>(width / 3)));
  for (int step = 0; step < steps; ++step) {
    const float segment = static_cast<float>(width) / static_cast<float>(steps);
    const float a0 = (angle - width + step * segment) * 0.01745329252f;
    const float a1 = (angle - width + (step + 1) * segment) * 0.01745329252f;
    const int r0 = 16;
    const int r1 = kProjectionRadius;
    const float t = static_cast<float>(step + 1) / static_cast<float>(steps);
    const uint8_t alpha = static_cast<uint8_t>(min(42, 4 + static_cast<int>(t * t * 38)));
    const int x00 = cx + static_cast<int>(cosf(a0) * r0);
    const int y00 = cy + static_cast<int>(sinf(a0) * r0);
    const int x01 = cx + static_cast<int>(cosf(a1) * r0);
    const int y01 = cy + static_cast<int>(sinf(a1) * r0);
    const int x10 = cx + static_cast<int>(cosf(a0) * r1);
    const int y10 = cy + static_cast<int>(sinf(a0) * r1);
    const int x11 = cx + static_cast<int>(cosf(a1) * r1);
    const int y11 = cy + static_cast<int>(sinf(a1) * r1);
    drawFilledQuadAlpha(x00, y00, x10, y10, x11, y11, x01, y01, color, alpha);
  }
}

void drawSweepTrail(float angle, uint16_t color, uint8_t widthDeg) {
  const uint8_t width = max<uint8_t>(8, min<uint8_t>(90, widthDeg));
  const uint8_t bands = 5;
  const float step = static_cast<float>(width) / static_cast<float>(bands);
  const int inner = 0;
  for (uint8_t i = 0; i < bands; ++i) {
    const float nearOffset = step * static_cast<float>(i);
    const float farOffset = step * static_cast<float>(i + 1);
    const float aNear = (angle - nearOffset) * 0.01745329252f;
    const float aFar = (angle - farOffset) * 0.01745329252f;
    const int outer = kProjectionRadius;
    const int x00 = 240 + static_cast<int>(cosf(aFar) * inner);
    const int y00 = 240 + static_cast<int>(sinf(aFar) * inner);
    const int x01 = 240 + static_cast<int>(cosf(aNear) * inner);
    const int y01 = 240 + static_cast<int>(sinf(aNear) * inner);
    const int x10 = 240 + static_cast<int>(cosf(aFar) * outer);
    const int y10 = 240 + static_cast<int>(sinf(aFar) * outer);
    const int x11 = 240 + static_cast<int>(cosf(aNear) * outer);
    const int y11 = 240 + static_cast<int>(sinf(aNear) * outer);
    const uint8_t alpha = static_cast<uint8_t>(52 - i * 9);
    drawFilledQuadAlpha(x00, y00, x10, y10, x11, y11, x01, y01, color, alpha);
  }
  const float a = angle * 0.01745329252f;
  const float ux = cosf(a);
  const float uy = sinf(a);
  const float nx = -uy;
  const float ny = ux;
  const int x1 = 240;
  const int y1 = 240;
  const int x2 = 240 + static_cast<int>(ux * kProjectionRadius);
  const int y2 = 240 + static_cast<int>(uy * kProjectionRadius);
  drawLineAlpha(x1, y1, x2, y2, color, 225);
  drawLineAlpha(x1 + static_cast<int>(nx), y1 + static_cast<int>(ny),
                x2 + static_cast<int>(nx), y2 + static_cast<int>(ny), color, 75);
  drawLineAlpha(x1 - static_cast<int>(nx), y1 - static_cast<int>(ny),
                x2 - static_cast<int>(nx), y2 - static_cast<int>(ny), color, 55);
}

void drawCrosshair(uint16_t color) {
  const uint8_t thickness = max(1, min(5, static_cast<int>(display_state.crosshairThickness)));
  if (display_state.crosshairStyle == 1) {
    drawDashedLineAlpha(240, 14, 240, 466, color, 100, 10, 10);
    drawDashedLineAlpha(14, 240, 466, 240, color, 100, 10, 10);
    if (thickness > 1) {
      drawDashedLineAlpha(239, 14, 239, 466, color, 65, 10, 10);
      drawDashedLineAlpha(241, 14, 241, 466, color, 65, 10, 10);
      drawDashedLineAlpha(14, 239, 466, 239, color, 65, 10, 10);
      drawDashedLineAlpha(14, 241, 466, 241, color, 65, 10, 10);
    }
  } else {
    drawWideLineAlpha(240, 14, 240, 466, color, 90, thickness);
    drawWideLineAlpha(14, 240, 466, 240, color, 90, thickness);
  }
}

void drawDigit(int x, int y, int digit, uint16_t color, int scale = 3) {
  static const uint8_t font[10][5] = {
      {0b111, 0b101, 0b101, 0b101, 0b111},
      {0b010, 0b110, 0b010, 0b010, 0b111},
      {0b111, 0b001, 0b111, 0b100, 0b111},
      {0b111, 0b001, 0b111, 0b001, 0b111},
      {0b101, 0b101, 0b111, 0b001, 0b001},
      {0b111, 0b100, 0b111, 0b001, 0b111},
      {0b111, 0b100, 0b111, 0b101, 0b111},
      {0b111, 0b001, 0b010, 0b010, 0b010},
      {0b111, 0b101, 0b111, 0b101, 0b111},
      {0b111, 0b101, 0b111, 0b001, 0b111},
  };
  if (digit < 0 || digit > 9) return;
  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (font[digit][row] & (1 << (2 - col))) {
        fillRect(x + col * scale, y + row * scale, scale, scale, color);
      }
    }
  }
}

uint8_t glyphRow(char c, int row) {
  c = c >= 'a' && c <= 'z' ? c - 32 : c;
  switch (c) {
    case 'A': { static const uint8_t g[] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'B': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
    case 'C': { static const uint8_t g[] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return g[row]; }
    case 'E': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'G': { static const uint8_t g[] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}; return g[row]; }
    case 'H': { static const uint8_t g[] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case 'J': { static const uint8_t g[] = {0x07,0x02,0x02,0x02,0x12,0x12,0x0C}; return g[row]; }
    case 'K': { static const uint8_t g[] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
    case 'L': { static const uint8_t g[] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'Q': { static const uint8_t g[] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
    case 'R': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'V': { static const uint8_t g[] = {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04}; return g[row]; }
    case 'W': { static const uint8_t g[] = {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
    case 'X': { static const uint8_t g[] = {0x11,0x0A,0x04,0x04,0x04,0x0A,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[] = {0x11,0x0A,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'Z': { static const uint8_t g[] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
    case '0': { static const uint8_t g[] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
    case '1': { static const uint8_t g[] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
    case '2': { static const uint8_t g[] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
    case '3': { static const uint8_t g[] = {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case '4': { static const uint8_t g[] = {0x12,0x12,0x12,0x1F,0x02,0x02,0x02}; return g[row]; }
    case '5': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}; return g[row]; }
    case '6': { static const uint8_t g[] = {0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
    case '7': { static const uint8_t g[] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
    case '8': { static const uint8_t g[] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
    case '9': { static const uint8_t g[] = {0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}; return g[row]; }
    case '-': { static const uint8_t g[] = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case '.': { static const uint8_t g[] = {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}; return g[row]; }
    case '/': { static const uint8_t g[] = {0x01,0x02,0x02,0x04,0x08,0x08,0x10}; return g[row]; }
    case ':': { static const uint8_t g[] = {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}; return g[row]; }
    case '>': { static const uint8_t g[] = {0x10,0x08,0x04,0x02,0x04,0x08,0x10}; return g[row]; }
    case '<': { static const uint8_t g[] = {0x01,0x02,0x04,0x08,0x04,0x02,0x01}; return g[row]; }
    default: return 0;
  }
}

uint8_t styledGlyphRow(char c, int row) {
  c = c >= 'a' && c <= 'z' ? c - 32 : c;
  const uint8_t style = display_state.fontStyle > 2 ? 0 : display_state.fontStyle;
  if (style == 1) {
    switch (c) {
      case 'A': { static const uint8_t g[] = {0x04,0x0A,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
      case 'B': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
      case 'C': { static const uint8_t g[] = {0x0F,0x10,0x10,0x10,0x10,0x10,0x0F}; return g[row]; }
      case 'D': { static const uint8_t g[] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return g[row]; }
      case 'E': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1C,0x10,0x10,0x1F}; return g[row]; }
      case 'F': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1C,0x10,0x10,0x10}; return g[row]; }
      case 'G': { static const uint8_t g[] = {0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
      case 'H': { static const uint8_t g[] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
      case 'I': { static const uint8_t g[] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
      case 'J': { static const uint8_t g[] = {0x07,0x02,0x02,0x02,0x12,0x12,0x0C}; return g[row]; }
      case 'K': { static const uint8_t g[] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
      case 'L': { static const uint8_t g[] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
      case 'M': { static const uint8_t g[] = {0x11,0x1B,0x15,0x11,0x11,0x11,0x11}; return g[row]; }
      case 'N': { static const uint8_t g[] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
      case 'O': { static const uint8_t g[] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
      case 'P': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
      case 'Q': { static const uint8_t g[] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
      case 'R': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
      case 'S': { static const uint8_t g[] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
      case 'T': { static const uint8_t g[] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
      case 'U': { static const uint8_t g[] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
      case 'V': { static const uint8_t g[] = {0x11,0x11,0x11,0x0A,0x0A,0x0A,0x04}; return g[row]; }
      case 'W': { static const uint8_t g[] = {0x11,0x11,0x11,0x15,0x15,0x15,0x0A}; return g[row]; }
      case 'X': { static const uint8_t g[] = {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
      case 'Y': { static const uint8_t g[] = {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
      case 'Z': { static const uint8_t g[] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
      case '0': { static const uint8_t g[] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
      case '1': { static const uint8_t g[] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
      case '2': { static const uint8_t g[] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
      case '3': { static const uint8_t g[] = {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[row]; }
      case '4': { static const uint8_t g[] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
      case '5': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1E,0x01,0x11,0x0E}; return g[row]; }
      case '6': { static const uint8_t g[] = {0x07,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
      case '7': { static const uint8_t g[] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
      case '8': { static const uint8_t g[] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
      case '9': { static const uint8_t g[] = {0x0E,0x11,0x11,0x0F,0x01,0x02,0x1C}; return g[row]; }
      default: return glyphRow(c, row);
    }
  }
  if (style == 2) {
    switch (c) {
      case 'A': { static const uint8_t g[] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
      case 'B': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
      case 'C': { static const uint8_t g[] = {0x0F,0x10,0x10,0x10,0x10,0x10,0x0F}; return g[row]; }
      case 'D': { static const uint8_t g[] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return g[row]; }
      case 'E': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
      case 'F': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
      case 'G': { static const uint8_t g[] = {0x0F,0x10,0x10,0x13,0x11,0x11,0x0F}; return g[row]; }
      case 'H': { static const uint8_t g[] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
      case 'I': { static const uint8_t g[] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x1F}; return g[row]; }
      case 'J': { static const uint8_t g[] = {0x07,0x02,0x02,0x02,0x12,0x12,0x0C}; return g[row]; }
      case 'K': { static const uint8_t g[] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
      case 'L': { static const uint8_t g[] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
      case 'M': { static const uint8_t g[] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
      case 'N': { static const uint8_t g[] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
      case 'O': { static const uint8_t g[] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
      case 'P': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
      case 'Q': { static const uint8_t g[] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
      case 'R': { static const uint8_t g[] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
      case 'S': { static const uint8_t g[] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
      case 'T': { static const uint8_t g[] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
      case 'U': { static const uint8_t g[] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
      case 'V': { static const uint8_t g[] = {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04}; return g[row]; }
      case 'W': { static const uint8_t g[] = {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}; return g[row]; }
      case 'X': { static const uint8_t g[] = {0x11,0x0A,0x04,0x04,0x04,0x0A,0x11}; return g[row]; }
      case 'Y': { static const uint8_t g[] = {0x11,0x0A,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
      case 'Z': { static const uint8_t g[] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
      case '0': { static const uint8_t g[] = {0x1F,0x11,0x13,0x15,0x19,0x11,0x1F}; return g[row]; }
      case '1': { static const uint8_t g[] = {0x04,0x0C,0x14,0x04,0x04,0x04,0x1F}; return g[row]; }
      case '2': { static const uint8_t g[] = {0x1F,0x01,0x01,0x1F,0x10,0x10,0x1F}; return g[row]; }
      case '3': { static const uint8_t g[] = {0x1F,0x01,0x01,0x0F,0x01,0x01,0x1F}; return g[row]; }
      case '4': { static const uint8_t g[] = {0x11,0x11,0x11,0x1F,0x01,0x01,0x01}; return g[row]; }
      case '5': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1F,0x01,0x01,0x1F}; return g[row]; }
      case '6': { static const uint8_t g[] = {0x1F,0x10,0x10,0x1F,0x11,0x11,0x1F}; return g[row]; }
      case '7': { static const uint8_t g[] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
      case '8': { static const uint8_t g[] = {0x1F,0x11,0x11,0x1F,0x11,0x11,0x1F}; return g[row]; }
      case '9': { static const uint8_t g[] = {0x1F,0x11,0x11,0x1F,0x01,0x01,0x1F}; return g[row]; }
      default: return glyphRow(c, row);
    }
  }
  return glyphRow(c, row);
}

uint8_t glyphPixelWidth(char c) {
  c = c >= 'a' && c <= 'z' ? c - 32 : c;
  const uint8_t style = display_state.fontStyle > 2 ? 0 : display_state.fontStyle;
  if (style != 1) return 5;
  switch (c) {
    case 'I':
    case '1':
    case '.':
      return 3;
    case 'J':
    case 'L':
    case 'T':
    case '/':
    case '-':
      return 4;
    case 'M':
    case 'W':
      return 5;
    default:
      return 5;
  }
}

uint8_t textAdvance(char c) {
  const uint8_t style = display_state.fontStyle > 2 ? 0 : display_state.fontStyle;
  return glyphPixelWidth(c) + (style == 1 ? 1 : 1);
}

void drawGlyphCell(int x, int y, int w, int h, uint16_t color, uint8_t style) {
  if (style == 1) {
    fillRectAlpha(x, y, w, h, color, 235);
  } else if (style == 2) {
    fillRectAlpha(x, y, w, h, color, 230);
    if (w > 1 && h > 1) {
      blendPixel(x, y, color, 120);
      blendPixel(x + w - 1, y, color, 120);
      blendPixel(x, y + h - 1, color, 120);
      blendPixel(x + w - 1, y + h - 1, color, 120);
    }
  } else {
    fillRect(x, y, w, h, color);
  }
}

void drawChar(int x, int y, char c, uint16_t color, int scale = 1) {
  const uint8_t style = display_state.fontStyle > 2 ? 0 : display_state.fontStyle;
  const uint8_t width = glyphPixelWidth(c);
  for (int row = 0; row < 7; ++row) {
    const uint8_t bits = styledGlyphRow(c, row);
    for (int col = 0; col < width; ++col) {
      if (bits & (1 << (4 - col))) drawGlyphCell(x + col * scale, y + row * scale, scale, scale, color, style);
    }
  }
}

void drawText(int x, int y, const char* text, uint16_t color, int scale = 1) {
  int cursor = x;
  while (*text) {
    if (*text != ' ') drawChar(cursor, y, *text, color, scale);
    cursor += textAdvance(*text) * scale;
    ++text;
  }
}

int textWidth(const char* text, int scale = 1) {
  int w = 0;
  while (text && *text) {
    w += textAdvance(*text) * scale;
    ++text;
  }
  return w;
}

int textWidthPercent(const char* text, uint8_t scalePercent) {
  int w = 0;
  while (text && *text) {
    w += max(3, (textAdvance(*text) * static_cast<int>(scalePercent) + 50) / 100);
    ++text;
  }
  return w;
}

void drawTextSoft(int x, int y, const char* value, uint16_t color, int scale = 1) {
  drawText(x + 1, y + 1, value, rgb565(0, 0, 0), scale);
  drawText(x, y, value, color, scale);
}

void drawCharPercent(int x, int y, char c, uint16_t color, uint8_t scalePercent) {
  const int glyphW = glyphPixelWidth(c);
  const int sx = max(1, (glyphW * static_cast<int>(scalePercent) + 50) / 100);
  const int sy = max(1, (7 * static_cast<int>(scalePercent) + 50) / 100);
  const uint8_t style = display_state.fontStyle > 2 ? 0 : display_state.fontStyle;
  for (int yy = 0; yy < sy; ++yy) {
    const int srcRow = min(6, yy * 7 / sy);
    const uint8_t bits = styledGlyphRow(c, srcRow);
    for (int xx = 0; xx < sx; ++xx) {
      const int srcCol = min(4, xx * 5 / sx);
      if (bits & (1 << (4 - srcCol))) {
        if (style == 2) {
          blendPixel(x + xx, y + yy, color, 220);
        } else {
          putPixel(x + xx, y + yy, color);
        }
      }
    }
  }
}

void drawTextPercent(int x, int y, const char* text, uint16_t color, uint8_t scalePercent) {
  int cursor = x;
  while (*text) {
    if (*text != ' ') drawCharPercent(cursor, y, *text, color, scalePercent);
    cursor += max(3, (textAdvance(*text) * static_cast<int>(scalePercent) + 50) / 100);
    ++text;
  }
}

void drawTextSoftPercent(int x, int y, const char* value, uint16_t color, uint8_t scalePercent) {
  drawTextPercent(x + 1, y + 1, value, rgb565(0, 0, 0), scalePercent);
  drawTextPercent(x, y, value, color, scalePercent);
}

void drawLabelBackplate(int x, int y, const char* value, uint8_t scalePercent, int detailLines, bool selected, bool light, uint8_t opacityPercent) {
  const int w = textWidthPercent(value, scalePercent) + 10;
  const int lineStep = max(8, (10 * static_cast<int>(scalePercent) + 50) / 100);
  const int glyphHeight = max(7, (7 * static_cast<int>(scalePercent) + 50) / 100);
  const int h = glyphHeight + 6 + detailLines * lineStep;
  const uint16_t fill = light ? rgb565(8, 22, 20) : rgb565(2, 11, 10);
  const uint16_t edge = light ? rgb565(42, 98, 80) : rgb565(16, 54, 42);
  const uint8_t alpha = selected ? 205 : static_cast<uint8_t>(min(145, max(0, static_cast<int>(opacityPercent)) * 145 / 100));
  const int bw = max(w, 50);
  if (alpha > 0) {
    fillRectAlpha(x - 5, y - 3, bw, h, rgb565(0, 0, 0), min(160, alpha + 35));
    fillRectAlpha(x - 4, y - 4, bw, h, fill, alpha);
    drawLineAlpha(x - 2, y - 4, x + bw - 8, y - 4, edge, selected ? 210 : 95);
    drawLineAlpha(x - 2, y + h - 4, x + bw - 8, y + h - 4, edge, selected ? 135 : 45);
  }
  if (selected) drawLineAlpha(x - 1, y - 6, x + bw - 12, y - 6, rgb565(40, 242, 110), 190);
}

void drawNumber(int x, int y, uint16_t number, uint16_t color, int scale = 3) {
  char buf[8] = {};
  snprintf(buf, sizeof(buf), "%u", number);
  int cursor = x;
  for (const char* p = buf; *p; ++p) {
    drawDigit(cursor, y, *p - '0', color, scale);
    cursor += 4 * scale;
  }
}

void drawNumberSoft(int x, int y, uint16_t number, uint16_t color, int scale = 3) {
  drawNumber(x + 1, y + 1, number, rgb565(0, 0, 0), scale);
  drawNumber(x, y, number, color, scale);
}

void drawScopeText(int x, int y, const char* value, uint16_t color, int scale = 2) {
  drawTextSoft(x + 1, y + 1, value, rgb565(0, 25, 11), scale);
  drawTextSoft(x, y, value, color, scale);
}

void drawStatusOverlay() {
  const char* value = nullptr;
  uint16_t color = rgb565(255, 34, 34);
  if (!display_state.wifiConnected) {
    value = "NO WIFI";
  } else if (display_state.mapLoading) {
    value = "MAP LOADING";
    color = rgb565(255, 214, 48);
  }
  if (!value) return;
  const int scale = 2;
  const int w = textWidth(value, scale) + 24;
  const int x = max(16, 240 - w / 2);
  const int y = 92;
  fillRectAlpha(x, y - 5, w, 28, rgb565(0, 0, 0), 255);
  drawTextSoft(x + 12, y, value, color, scale);
}

void drawCenteredScopeText(int y, const char* value, uint16_t color, int scale = 2) {
  const int w = textWidth(value, scale);
  drawScopeText(max(8, 240 - w / 2), y, value, color, scale);
}

void drawPill(int x, int y, int w, int h, uint16_t border, uint16_t fill) {
  const int r = h / 2;
  fillRect(x + r, y, w - h, h, fill);
  drawFilledCircle(x + r, y + r, r, fill);
  drawFilledCircle(x + w - r, y + r, r, fill);
  drawCircle(x + r, y + r, r, border);
  drawCircle(x + w - r, y + r, r, border);
  fillRect(x + r, y, w - h, 2, border);
  fillRect(x + r, y + h - 2, w - h, 2, border);
}

void rotatedPoint(int cx, int cy, float forward, float side, float heading, float scale, int* outX, int* outY) {
  const float ux = cosf(heading);
  const float uy = sinf(heading);
  const float sx = -uy;
  const float sy = ux;
  *outX = cx + static_cast<int>(roundf((ux * forward + sx * side) * scale));
  *outY = cy + static_cast<int>(roundf((uy * forward + sy * side) * scale));
}

void drawAircraftSilhouette(int cx, int cy, float heading, float scale, uint16_t color, uint8_t alpha) {
  int noseX, noseY, bodyLX, bodyLY, bodyRX, bodyRY, tailX, tailY;
  int wingLX, wingLY, wingRX, wingRY, wingBackLX, wingBackLY, wingBackRX, wingBackRY;
  int tailLX, tailLY, tailRX, tailRY;
  rotatedPoint(cx, cy, 15, 0, heading, scale, &noseX, &noseY);
  rotatedPoint(cx, cy, -8, 0, heading, scale, &tailX, &tailY);
  rotatedPoint(cx, cy, 6, -3, heading, scale, &bodyLX, &bodyLY);
  rotatedPoint(cx, cy, 6, 3, heading, scale, &bodyRX, &bodyRY);
  rotatedPoint(cx, cy, 1, -12, heading, scale, &wingLX, &wingLY);
  rotatedPoint(cx, cy, 1, 12, heading, scale, &wingRX, &wingRY);
  rotatedPoint(cx, cy, -4, -4, heading, scale, &wingBackLX, &wingBackLY);
  rotatedPoint(cx, cy, -4, 4, heading, scale, &wingBackRX, &wingBackRY);
  rotatedPoint(cx, cy, -9, -6, heading, scale, &tailLX, &tailLY);
  rotatedPoint(cx, cy, -9, 6, heading, scale, &tailRX, &tailRY);
  drawFilledTriangleAlpha(noseX, noseY, wingLX, wingLY, wingBackLX, wingBackLY, color, alpha);
  drawFilledTriangleAlpha(noseX, noseY, wingBackRX, wingBackRY, wingRX, wingRY, color, alpha);
  drawFilledTriangleAlpha(noseX, noseY, bodyLX, bodyLY, tailX, tailY, color, alpha);
  drawFilledTriangleAlpha(noseX, noseY, tailX, tailY, bodyRX, bodyRY, color, alpha);
  drawFilledTriangleAlpha(tailX, tailY, tailLX, tailLY, bodyLX, bodyLY, color, static_cast<uint8_t>(alpha * 7 / 10));
  drawFilledTriangleAlpha(tailX, tailY, bodyRX, bodyRY, tailRX, tailRY, color, static_cast<uint8_t>(alpha * 7 / 10));
  drawLineAlpha(noseX, noseY, tailX, tailY, boost565(color, 135), min(255, alpha + 35));
}

void drawAircraftMarker(const MicroRadarDisplayAircraft& ac, uint16_t color) {
  const int cx = 240 + ac.x;
  const int cy = 240 + ac.y;
  if (cx < -42 || cx >= kWidth + 42 || cy < -42 || cy >= kHeight + 42) return;
  const float scale = max(0.70f, min(1.60f, display_state.aircraftIconScale ? display_state.aircraftIconScale / 100.0f : display_state.uiScale / 100.0f));

  const uint8_t iconType = ac.iconType < (sizeof(kAircraftSprites) / sizeof(kAircraftSprites[0])) ? ac.iconType : AIRCRAFT_ICON_NARROW_JET;
  const AircraftSpriteDef& sprite = kAircraftSprites[iconType];
  int heading = ac.headingDeg % 360;
  if (heading < 0) heading += 360;
  const uint8_t rotationIndex = static_cast<uint8_t>(((heading * sprite.rotations) + 180) / 360) % sprite.rotations;
  const uint32_t rotationOffset = static_cast<uint32_t>(rotationIndex) * sprite.w * sprite.h;
  const int halfW = static_cast<int>(ceilf(sprite.w * scale * 0.5f));
  const int halfH = static_cast<int>(ceilf(sprite.h * scale * 0.5f));
  const float invScale = 1.0f / scale;

  for (int dy = -halfH; dy <= halfH; ++dy) {
    const int py = cy + dy;
    if (py < 0 || py >= kHeight) continue;
    for (int dx = -halfW; dx <= halfW; ++dx) {
      const int px = cx + dx;
      if (px < 0 || px >= kWidth) continue;
      const float sx = dx * invScale + (sprite.w - 1) * 0.5f;
      const float sy = dy * invScale + (sprite.h - 1) * 0.5f;
      const int ix = static_cast<int>(sx + 0.5f);
      const int iy = static_cast<int>(sy + 0.5f);
      if (ix < 0 || ix >= sprite.w || iy < 0 || iy >= sprite.h) continue;
      const uint32_t idx = rotationOffset + iy * sprite.w + ix;
      const uint8_t alpha = sprite.alpha[idx];
      if (alpha < 18) continue;
      uint16_t pixel = sprite.rgb[idx];
      if (ac.emergencyActive) pixel = blend565(pixel, rgb565(255, 24, 24), 185);
      else if (ac.stale) pixel = blend565(pixel, rgb565(95, 100, 96), 165);
      else if (color != rgb888To565(0xF5EC00)) pixel = tint565PreserveLight(pixel, color);
      blendPixel(px + 2, py + 2, rgb565(0, 0, 0), static_cast<uint8_t>(alpha / 3));
      blendPixel(px, py, pixel, alpha);
    }
  }
  if (ac.selected) {
    drawCircleAlpha(cx, cy, static_cast<int>(22 * scale), rgb565(255, 184, 77), 210);
    drawCircleAlpha(cx, cy, static_cast<int>(26 * scale), rgb565(255, 184, 77), 90);
  }
  if (ac.tracked) {
    drawCircleAlpha(cx, cy, static_cast<int>(30 * scale), rgb565(255, 244, 168), 140);
  }
}

void drawCenteredText(int y, const char* textValue, uint16_t color, int scale = 1) {
  const int w = textWidth(textValue, scale);
  drawTextSoft(max(8, 240 - w / 2), y, textValue, color, scale);
}

void drawCenteredTextOffset(int y, const char* textValue, uint16_t color, int scale, int xOffset) {
  const int w = textWidth(textValue, scale);
  drawTextSoft(max(8, 240 - w / 2 + xOffset), y, textValue, color, scale);
}

void drawDetailRow(int y, const char* label, const char* value, uint16_t labelColor, uint16_t valueColor) {
  constexpr int kScale = 2;
  drawTextSoft(72, y, label, labelColor, kScale);
  if (value && value[0] != '\0') {
    drawTextSoft(200, y, value, valueColor, kScale);
  } else {
    drawTextSoft(200, y, "-", valueColor, kScale);
  }
}

void drawDetailMetricBox(int x, int y, int w, const char* label, const char* value, uint16_t accent, uint16_t valueColor) {
  fillRoundRectAlpha(x, y, w, 52, 7, rgb565(3, 16, 15), 178);
  drawLineAlpha(x + 8, y + 5, x + w - 10, y + 5, boost565(accent, 80), 120);
  drawTextSoft(x + 10, y + 10, label, boost565(accent, 72), 1);
  drawTextSoft(x + 10, y + 28, value && value[0] ? value : "-", valueColor, 2);
}

void drawDetailTextLine(int x, int y, const char* label, const char* value, uint16_t labelColor, uint16_t valueColor) {
  drawTextSoft(x, y, label, labelColor, 1);
  drawTextSoft(x + 94, y - 3, value && value[0] ? value : "-", valueColor, 2);
}

void drawAircraftDetailPage() {
  const uint16_t bg = rgb888To565(display_state.scopeOutsideColor);
  const uint16_t panelBg = rgb888To565(display_state.detailBackgroundColor);
  const uint16_t accent = rgb888To565(display_state.labelColor);
  const uint16_t detail = rgb888To565(display_state.detailLabelColor);
  const uint16_t emergencyRed = rgb565(255, 32, 32);
  const uint16_t muted = rgb565(118, 146, 136);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const int dx = x - 240;
      const int dy = y - 240;
      putPixel(x, y, (dx * dx + dy * dy <= kScopeRadius * kScopeRadius) ? panelBg : bg);
    }
  }
  for (int r = kScopeEdgeRadius - 10; r <= kScopeEdgeRadius; r += 5) drawCircleAlpha(240, 240, r, accent, r == kScopeEdgeRadius ? 135 : 35);
  drawCircleAlpha(240, 240, 176, detail, 22);
  drawCircleAlpha(240, 240, 118, detail, 18);
  const MicroRadarDisplayAircraft* selected = nullptr;
  for (uint16_t i = 0; i < display_state.aircraftRendered && i < MICRO_RADAR_MAX_DISPLAY_AIRCRAFT; ++i) {
    if (display_state.aircraft[i].selected) {
      selected = &display_state.aircraft[i];
      break;
    }
  }
  if (!selected) {
    drawCenteredText(190, "NO AIRCRAFT SELECTED", muted, 1);
  } else {
    const uint16_t activeAccent = selected->emergencyActive ? emergencyRed : detail;
    const uint16_t valueColor = accent;
    drawCenteredText(44, selected->emergencyActive ? "EMERGENCY" : "AIRCRAFT", selected->emergencyActive ? emergencyRed : detail, 1);
    drawCenteredTextOffset(64, selected->label[0] ? selected->label : selected->icao, activeAccent, 3, 3);

    char altitude[18] = {};
    char speed[18] = {};
    char heading[18] = {};
    char squawk[18] = {};
    char airline[22] = {};
    char route[18] = {};
    char identity[32] = {};
    if (selected->altitudeFt > 0) snprintf(altitude, sizeof(altitude), "FL%ld", static_cast<long>(selected->altitudeFt / 100));
    if (selected->speedKt > 0) snprintf(speed, sizeof(speed), "%u KT", static_cast<unsigned>(selected->speedKt));
    snprintf(heading, sizeof(heading), "%d DEG", static_cast<int>(selected->headingDeg));
    if (selected->squawk[0] != '\0') snprintf(squawk, sizeof(squawk), "%s", selected->squawk);
    if (selected->airline[0] != '\0' || selected->operatorName[0] != '\0') {
      snprintf(airline, sizeof(airline), "%s", selected->airline[0] ? selected->airline : selected->operatorName);
    } else {
      snprintf(airline, sizeof(airline), "LOADING...");
    }
    if (selected->routeOrigin[0] != '\0' || selected->routeDestination[0] != '\0') {
      snprintf(route, sizeof(route), "%s > %s", selected->routeOrigin[0] ? selected->routeOrigin : "?", selected->routeDestination[0] ? selected->routeDestination : "?");
    } else {
      snprintf(route, sizeof(route), "LOADING...");
    }
    snprintf(identity, sizeof(identity), "%s", selected->registration[0] ? selected->registration : selected->icao);

    drawLineAlpha(72, 104, 408, 104, activeAccent, 105);
    drawDetailRow(128, "AIRLINE", airline, detail, valueColor);
    drawDetailRow(164, "MODEL", selected->modelName[0] ? selected->modelName : selected->typeCode, detail, valueColor);
    drawDetailRow(200, "ROUTE", route, detail, valueColor);
    drawDetailRow(236, "ID", identity, detail, valueColor);
    drawDetailRow(272, "ALT", altitude, detail, valueColor);
    drawDetailRow(308, "SPD", speed, detail, valueColor);
    drawDetailRow(344, "HDG", heading, detail, valueColor);
    drawDetailRow(380, "SQUAWK", squawk, detail, selected->emergencyActive ? emergencyRed : valueColor);
    if (selected->emergencyActive) {
      drawDetailRow(406, "STATUS", selected->emergency[0] ? selected->emergency : "EMERGENCY", detail, emergencyRed);
    }
  }
  drawLineAlpha(174, 421, 306, 421, accent, 85);
  drawCenteredText(430, "DISMISS", accent, 2);
}

void drawScopeBezel(uint16_t edge, uint16_t dim, uint16_t text, bool showOuterRangeRing, bool showScopeEdge) {
  if (!showScopeEdge) return;
  const uint8_t thickness = max<uint8_t>(1, min<uint8_t>(5, display_state.scopeEdgeThickness ? display_state.scopeEdgeThickness : 1));
  drawThickCircleAlpha(240, 240, kScopeEdgeRadius, rgb565(0, 0, 0), 170, thickness + 1);
  drawThickCircleAlpha(240, 240, kScopeEdgeRadius - 1, edge, 150, thickness);
  if (showOuterRangeRing) drawCircleAlpha(240, 240, kProjectionRadius, edge, 65);
  for (int deg = 0; deg < 360; deg += 10) {
    const float a = deg * 0.01745329252f;
    const int outer = kScopeEdgeRadius - 4;
    const int inner = outer - (deg % 30 == 0 ? 10 : 5);
    const uint8_t alpha = deg % 30 == 0 ? 110 : 55;
    drawLineAlpha(240 + static_cast<int>(cosf(a) * inner), 240 + static_cast<int>(sinf(a) * inner),
                  240 + static_cast<int>(cosf(a) * outer), 240 + static_cast<int>(sinf(a) * outer), deg % 90 == 0 ? text : dim, alpha);
  }
}

void drawFrame() {
  if (display_state.detailOpen) {
    const uint32_t currentDetailHash = detailLayerHash();
    applyBacklightBrightness(display_state.brightness ? display_state.brightness : 255);
    if (!detail_cache_valid || detail_cache_hash != currentDetailHash) {
      drawAircraftDetailPage();
      applySoftwareBrightnessFallback(display_state.brightness ? display_state.brightness : 255);
      esp_lcd_panel_draw_bitmap(panel, 0, 0, kWidth, kHeight, framebuffer);
      detail_cache_hash = currentDetailHash;
      detail_cache_valid = true;
      ++full_flushes;
    }
    previous_dynamic_rect_valid = false;
    return;
  }
  detail_cache_valid = false;
  const bool light = display_state.lightTheme;
  const uint32_t sweepRgb = display_state.sweepColor;
  const uint32_t aircraftRgb = display_state.aircraftColor;
  const uint32_t labelRgb = display_state.labelColor;
  const uint32_t detailLabelRgb = display_state.detailLabelColor;
  const uint32_t altitudeLabelRgb = display_state.altitudeLabelColor;
  const uint32_t speedLabelRgb = display_state.speedLabelColor;
  const uint32_t airportRgb = display_state.airportColor;
  const uint32_t airportLabelRgb = display_state.airportLabelColor;
  const uint16_t water = rgb888To565(display_state.waterColor);
  const uint16_t land = rgb888To565(display_state.landColor);
  const uint16_t scopeBg = water;
  const uint16_t outerBg = rgb888To565(display_state.scopeOutsideColor);
  const uint16_t bg = display_state.mapEnabled ? scopeBg : scopeBg;
  const uint16_t scopeColor = rgb888To565(display_state.scopeBackgroundColor);
  const uint16_t grid = scopeColor;
  const uint16_t sweep = rgb888To565(sweepRgb);
  const uint16_t aircraftYellow = rgb888To565(aircraftRgb);
  const uint16_t text = rgb888To565(labelRgb);
  const uint16_t detailText = rgb888To565(detailLabelRgb);
  const uint16_t altitudeText = rgb888To565(altitudeLabelRgb);
  const uint16_t speedText = rgb888To565(speedLabelRgb);
  const uint16_t airportLabel = rgb888To565(airportLabelRgb);
  const uint16_t amber = text;
  const uint16_t dim = scopeColor;
  const uint16_t trail = rgb888To565(display_state.trailColor);
  const uint16_t stale = rgb565(120, 125, 120);
  const uint16_t mapCoast = rgb888To565(display_state.mapCoastColor);
  const uint16_t mapBorder = rgb888To565(display_state.mapBorderColor);
  const uint16_t mapWaterLine = rgb888To565(display_state.mapWaterLineColor);
  const uint16_t crosshair = scopeColor;

  const uint32_t currentStaticHash = staticLayerHash();
  const bool cacheHit = static_cache_available && static_cache_valid && static_cache_hash == currentStaticHash;
  DirtyRect dirty;
  DirtyRect currentDynamic;
  includeDynamicRect(&currentDynamic, true);
  if (cacheHit) {
    ++static_cache_hits;
    dirty = currentDynamic;
    if (previous_dynamic_rect_valid && previous_dynamic_rect.valid) {
      includeRect(&dirty, previous_dynamic_rect.x0, previous_dynamic_rect.y0, previous_dynamic_rect.x1, previous_dynamic_rect.y1);
    }
    restoreRectFromStatic(dirty);
  } else {
    ++static_cache_misses;
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        int dx = x - 240;
        int dy = y - 240;
        int d2 = dx * dx + dy * dy;
        if (d2 <= kScopeRadius * kScopeRadius) {
          uint16_t base = bg;
          if (display_state.mapEnabled && display_state.rasterBackgroundReady && display_state.rasterBackground) {
            base = display_state.rasterBackground[y * kWidth + x];
            const uint8_t mapBrightness = display_state.mapBrightness > 100 ? 100 : display_state.mapBrightness;
            if (mapBrightness < 100) base = blend565(base, rgb565(0, 0, 0), 255 - static_cast<uint8_t>((mapBrightness * 255) / 100));
          }
          if (display_state.scopeEdgeEnabled && d2 > kProjectionRadius * kProjectionRadius) base = blend565(base, scopeColor, 48);
          else if (display_state.scopeEdgeEnabled && d2 > 206 * 206) base = blend565(base, rgb565(0, 0, 0), 26);
          putPixel(x, y, base);
        } else {
          putPixel(x, y, outerBg);
        }
      }
    }

    if (display_state.mapEnabled && display_state.mapSegmentCount > 0) {
      const uint16_t count = display_state.mapSegmentCount > MICRO_RADAR_MAX_DISPLAY_MAP_SEGMENTS ? MICRO_RADAR_MAX_DISPLAY_MAP_SEGMENTS : display_state.mapSegmentCount;
      int coastXByRow[kHeight];
      for (int i = 0; i < kHeight; ++i) coastXByRow[i] = -1;
      for (uint16_t i = 0; i < count; ++i) {
        const auto& s = display_state.mapSegments[i];
        if (s.kind == 1) markCoastRows(240 + s.x0, 240 + s.y0, 240 + s.x1, 240 + s.y1, coastXByRow);
      }
      fillLandFromCoastRows(coastXByRow, blend565(land, bg, 28));
      for (uint16_t i = 0; i < count; ++i) {
        const auto& s = display_state.mapSegments[i];
        if (s.kind != 1 && s.kind != 2) continue;
        const int x0 = 240 + s.x0;
        const int y0 = 240 + s.y0;
        const int x1 = 240 + s.x1;
        const int y1 = 240 + s.y1;
        if (s.kind == 1) {
          drawWideLineAlpha(x0, y0, x1, y1, rgb565(3, 8, 10), 105, 2);
          drawLineAlpha(x0, y0, x1, y1, mapCoast, 210);
        } else if (s.kind == 2) {
          drawDashedLineAlpha(x0, y0, x1, y1, mapBorder, 105, 8, 6);
        }
      }
    }
    if (display_state.mapEnabled && display_state.airportsEnabled && display_state.mapPointCount > 0) {
      const uint16_t airport = rgb888To565(airportRgb);
      const uint8_t count = display_state.mapPointCount > MICRO_RADAR_MAX_DISPLAY_MAP_POINTS ? MICRO_RADAR_MAX_DISPLAY_MAP_POINTS : display_state.mapPointCount;
      for (uint8_t i = 0; i < count; ++i) {
        const int px = 240 + display_state.mapPoints[i].x;
        const int py = 240 + display_state.mapPoints[i].y;
        if (px < 24 || px > 456 || py < 24 || py > 456) continue;
        if (display_state.airportIconType == 1) {
          drawFilledCircleAlpha(px, py, 4, airport, 210);
          drawCircleAlpha(px, py, 7, airport, 80);
        } else {
          if (display_state.airportIconType == 0) {
            drawFilledCircleAlpha(px, py, 11, bg, 205);
            drawCircleAlpha(px, py, 11, airport, 180);
          }
          drawWideLineAlpha(px - 8, py + 3, px + 8, py - 3, airport, 230, 2);
          drawLineAlpha(px - 3, py - 7, px + 3, py + 7, airport, 205);
          drawLineAlpha(px - 7, py - 3, px + 7, py + 3, airport, 150);
          drawFilledCircle(px, py, 2, airport);
        }
        if (display_state.airportLabelsEnabled && display_state.mapPoints[i].label[0] != '\0') {
          const uint8_t airportTextScale = max(70, min(160, display_state.airportLabelScale ? static_cast<int>(display_state.airportLabelScale) : 100));
          const int airportGlyphHeight = max(7, (7 * static_cast<int>(airportTextScale) + 50) / 100);
          const int tx = min(414, px + 14);
          const int ty = max(24, min(448, py - airportGlyphHeight / 2));
          drawTextSoftPercent(tx, ty, display_state.mapPoints[i].label, airportLabel, airportTextScale);
        }
      }
    }

    drawScopeBezel(grid, dim, text, display_state.rangeRingsEnabled && display_state.outerRangeRingEnabled, display_state.scopeEdgeEnabled);
    if (display_state.rangeRingsEnabled) {
      const int rings[] = {58, 116, 174, kProjectionRadius};
      const uint8_t thickness = max(1, min(5, static_cast<int>(display_state.rangeRingThickness)));
      for (uint8_t i = 0; i < 4; ++i) {
        if (i == 3 && (!display_state.outerRangeRingEnabled || !display_state.scopeEdgeEnabled)) continue;
        const int r = rings[i];
        if (display_state.rangeRingStyle == 1) {
        drawDashedCircleAlpha(240, 240, r, rgb565(0, 0, 0), 58, thickness + 1);
        drawDashedCircleAlpha(240, 240, r, grid, 112, thickness);
      } else {
        drawThickCircleAlpha(240, 240, r, rgb565(0, 0, 0), 58, thickness + 1);
        drawThickCircleAlpha(240, 240, r, grid, 112, thickness);
      }
        if (display_state.rangeRingLabelsEnabled) {
          const uint16_t value = display_state.metricUnits
                                     ? static_cast<uint16_t>((display_state.rangeNm * (i + 1) * 1852UL / 4UL + 500) / 1000)
                                     : static_cast<uint16_t>(display_state.rangeNm * (i + 1) / 4);
          char ringLabel[12] = {};
          snprintf(ringLabel, sizeof(ringLabel), "%u%s", value, display_state.metricUnits ? "km" : "nm");
          drawTextSoft(min(417, 247 + r), 228, ringLabel, grid, 1);
        }
      }
    }
    if (display_state.crosshairEnabled) {
      drawCrosshair(crosshair);
    }
    const uint16_t rangeValue = display_state.metricUnits ? static_cast<uint16_t>(display_state.rangeNm * 1852 / 1000) : display_state.rangeNm;
    char bottomRange[18] = {};
    snprintf(bottomRange, sizeof(bottomRange), "%u%s", rangeValue, display_state.metricUnits ? "km" : "nm");
    drawCenteredScopeText(424, bottomRange, text, 2);
    const uint16_t cardinalText = rgb888To565(display_state.cardinalLabelColor ? display_state.cardinalLabelColor : display_state.labelColor);
    const uint16_t ordinalText = rgb888To565(display_state.ordinalLabelColor ? display_state.ordinalLabelColor : display_state.labelColor);
    if (display_state.cardinalLabelsEnabled) {
      drawScopeText(235, 16, "N", cardinalText, 2);
      drawScopeText(235, 450, "S", cardinalText, 2);
      drawScopeText(18, 238, "W", cardinalText, 2);
      drawScopeText(438, 238, "E", cardinalText, 2);
    }
    if (display_state.ordinalLabelsEnabled) {
      drawScopeText(83, 78, "NW", ordinalText, 1);
      drawScopeText(382, 78, "NE", ordinalText, 1);
      drawScopeText(83, 393, "SW", ordinalText, 1);
      drawScopeText(382, 393, "SE", ordinalText, 1);
    }
    if (static_cache_available) {
      memcpy(static_framebuffer, framebuffer, kWidth * kHeight * sizeof(uint16_t));
      static_cache_hash = currentStaticHash;
      static_cache_valid = true;
    }
    includeRect(&dirty, 0, 0, kWidth, kHeight);
  }

  {
    const uint16_t rendered = display_state.aircraftRendered > MICRO_RADAR_MAX_DISPLAY_AIRCRAFT ? MICRO_RADAR_MAX_DISPLAY_AIRCRAFT : display_state.aircraftRendered;
    for (uint16_t i = 0; i < rendered; ++i) {
      if (display_state.trailsEnabled) {
        const uint8_t trailCount = display_state.aircraft[i].trailCount > 8 ? 8 : display_state.aircraft[i].trailCount;
        for (uint8_t j = 0; j < trailCount; ++j) {
          const int tx = 240 + display_state.aircraft[i].trailX[j];
          const int ty = 240 + display_state.aircraft[i].trailY[j];
          if (tx > 20 && tx < 460 && ty > 20 && ty < 460) {
            drawFilledCircleAlpha(tx, ty, j < 3 ? 3 : 2, trail, j < 3 ? 130 : 80);
            if (j < 2) drawCircleAlpha(tx, ty, 4 - j, trail, 80);
          }
        }
      }
      const uint16_t aircraftColor = display_state.aircraft[i].emergencyActive ? rgb565(255, 32, 32) : (display_state.aircraft[i].stale ? stale : aircraftYellow);
      drawAircraftMarker(display_state.aircraft[i], aircraftColor);
      const bool labelsOff = display_state.aircraftLabelMode == 3;
      const bool showLabel = !labelsOff && (display_state.aircraft[i].selected || (display_state.labelDensity > 0 && i < display_state.labelDensity * 4));
      if (showLabel && display_state.aircraft[i].label[0] != '\0') {
        const float iconScale = max(0.70f, min(1.60f, display_state.aircraftIconScale ? display_state.aircraftIconScale / 100.0f : display_state.uiScale / 100.0f));
        const uint8_t textScale = max(70, min(220, display_state.aircraftTextScale ? static_cast<int>(display_state.aircraftTextScale) : 100));
        const int lineStep = max(8, (10 * static_cast<int>(textScale) + 50) / 100);
        const int iconX = 240 + display_state.aircraft[i].x;
        const int iconY = 240 + display_state.aircraft[i].y;
        const int labelAnchorOffsetPx = max(12, min(60, static_cast<int>(display_state.aircraftLabelSpacing ? display_state.aircraftLabelSpacing : 28)));
        int lx = iconX + static_cast<int>(labelAnchorOffsetPx * iconScale);
        int ly = iconY - 16;
        ly = max(34, min(418, ly));
        if (lx > 36 && lx < 468 && ly > 36 && ly < 420) {
          const bool fullLabels = display_state.aircraftLabelMode == 2;
          const bool basicLabels = display_state.aircraftLabelMode == 1 || display_state.aircraft[i].selected;
          const int detailLines = (basicLabels || fullLabels)
                                      ? (display_state.aircraft[i].altitudeFt > 0 && display_state.aircraft[i].speedKt > 0 ? 2 : 1)
                                      : 0;
          drawLabelBackplate(lx, ly, display_state.aircraft[i].label, textScale, detailLines, display_state.aircraft[i].selected, light,
                             display_state.labelBackplateOpacity);
          drawLineAlpha(iconX + static_cast<int>(24 * iconScale), iconY, lx - 7, ly + 6,
                        display_state.aircraft[i].emergencyActive ? rgb565(255, 32, 32) : (display_state.aircraft[i].selected ? amber : text), display_state.aircraft[i].selected ? 135 : 70);
          const uint16_t primaryLabelColor = display_state.aircraft[i].emergencyActive ? rgb565(255, 32, 32) : (display_state.aircraft[i].selected ? amber : text);
          drawTextSoftPercent(lx, ly, display_state.aircraft[i].label, primaryLabelColor, textScale);
          if (basicLabels || fullLabels) {
            char altitude[10] = {};
            char speed[10] = {};
            if (display_state.aircraft[i].altitudeFt > 0) snprintf(altitude, sizeof(altitude), "FL%u", static_cast<unsigned>(display_state.aircraft[i].altitudeFt / 100));
            if (display_state.aircraft[i].speedKt > 0) snprintf(speed, sizeof(speed), "%uKT", static_cast<unsigned>(display_state.aircraft[i].speedKt));
            const uint16_t emergencyText = rgb565(255, 32, 32);
            if (altitude[0] != '\0') drawTextSoftPercent(lx, ly + lineStep, altitude,
                                                          display_state.aircraft[i].emergencyActive ? emergencyText : (display_state.aircraft[i].selected ? amber : altitudeText),
                                                          textScale);
            if (speed[0] != '\0') drawTextSoftPercent(lx, ly + lineStep * 2, speed,
                                                       display_state.aircraft[i].emergencyActive ? emergencyText : (display_state.aircraft[i].selected ? amber : speedText),
                                                       textScale);
          }
        }
      }
    }
  }

  if (display_state.sweepLineEnabled) {
    drawSweepTrail(sweep_angle, sweep, display_state.sweepFadeWidthDeg ? display_state.sweepFadeWidthDeg : 24);
  }
  drawStatusOverlay();

  applyBacklightBrightness(display_state.brightness ? display_state.brightness : 255);
  applySoftwareBrightnessFallback(display_state.brightness ? display_state.brightness : 255);
  if (display_state.brightness && display_state.brightness < 245) {
    esp_lcd_panel_draw_bitmap(panel, 0, 0, kWidth, kHeight, framebuffer);
    ++full_flushes;
  } else {
    flushRect(dirty);
  }
  previous_sweep_angle = sweep_angle;
  previous_sweep_valid = true;
  previous_dynamic_rect = currentDynamic;
  previous_dynamic_rect_valid = currentDynamic.valid;
}
}  // namespace

bool microRadarSmokeBegin() {
  if (panel && framebuffer) {
    Serial.println("Desktop Airspace display already initialized");
    return true;
  }
  Serial.println();
  Serial.println("Desktop Airspace display smoke test");
  Serial.printf("Renderer: %s\n", kRendererRevision);
  Serial.printf("Reset reason: %d\n", static_cast<int>(esp_reset_reason()));
  Serial.printf("PSRAM: %s, free=%u\n", psramFound() ? "yes" : "no", ESP.getFreePsram());

  framebuffer = static_cast<uint16_t*>(heap_caps_malloc(kWidth * kHeight * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!framebuffer) {
    Serial.println("PSRAM framebuffer allocation failed, trying internal memory");
    framebuffer = static_cast<uint16_t*>(heap_caps_malloc(kWidth * kHeight * sizeof(uint16_t), MALLOC_CAP_8BIT));
  }
  if (!framebuffer) {
    Serial.println("Framebuffer allocation failed");
    return false;
  }
  static_framebuffer = static_cast<uint16_t*>(heap_caps_malloc(kWidth * kHeight * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  static_cache_available = static_framebuffer != nullptr;
  static_cache_valid = false;
  Serial.printf("Static layer cache: %s\n", static_cache_available ? "enabled" : "unavailable");
  flush_stripe = static_cast<uint16_t*>(heap_caps_malloc(kWidth * 12 * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!flush_stripe) flush_stripe = static_cast<uint16_t*>(heap_caps_malloc(kWidth * 12 * sizeof(uint16_t), MALLOC_CAP_8BIT));
  Serial.printf("Partial flush stripe: %s\n", flush_stripe ? "enabled" : "unavailable");
  dirty_framebuffer = static_cast<uint16_t*>(heap_caps_malloc(kDirtyBufferMaxPixels * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  Serial.printf("Packed dirty buffer: %s\n", dirty_framebuffer ? "enabled" : "unavailable");

  Serial.println("Initialize ST7701 command interface");
  initBacklight();
  init3WireSpi();
  initSt7701();

  Serial.println("Initialize RGB panel");
  if (!initRgbPanel()) {
    Serial.println("RGB panel init failed");
    return false;
  }

  Serial.println("Drawing Desktop Airspace smoke screen");
  drawFrame();
  return true;
}

void microRadarSmokeTick() {
  if (panel && framebuffer) {
    const uint32_t now = millis();
    if (fps_window_ms == 0) fps_window_ms = now;
    if (last_sweep_ms == 0) last_sweep_ms = now;
    const uint32_t elapsed = now - last_sweep_ms;
    last_sweep_ms = now;
    const uint8_t seconds = display_state.sweepSecondsPerRotation < 1 ? 1 : display_state.sweepSecondsPerRotation > 10 ? 10 : display_state.sweepSecondsPerRotation;
    sweep_angle += 360.0f * static_cast<float>(elapsed) / (static_cast<float>(seconds) * 1000.0f);
    while (sweep_angle >= 360.0f) sweep_angle -= 360.0f;
    const uint32_t frameStartUs = micros();
    drawFrame();
    ++frame_counter;
    if (now - fps_window_ms >= 5000) {
      const uint32_t fpsWindow = now - fps_window_ms;
      measured_fps = static_cast<uint16_t>((frame_counter * 1000UL) / (fpsWindow == 0 ? 1UL : fpsWindow));
      Serial.printf("Render diag: fps=%u frame_us=%lu brightness=%u map_segments=%u map_points=%u aircraft=%u sweep_s=%u static_cache=%u hits=%lu misses=%lu partial=%lu full=%lu packed=%lu stripe=%lu\n",
                    measured_fps, static_cast<unsigned long>(micros() - frameStartUs),
                    display_state.brightness ? display_state.brightness : 255,
                    static_cast<unsigned>(display_state.mapSegmentCount),
                    static_cast<unsigned>(display_state.mapPointCount),
                    static_cast<unsigned>(display_state.aircraftRendered),
                    static_cast<unsigned>(seconds),
                    static_cache_available ? 1U : 0U,
                    static_cast<unsigned long>(static_cache_hits),
                    static_cast<unsigned long>(static_cache_misses),
                    static_cast<unsigned long>(partial_flushes),
                    static_cast<unsigned long>(full_flushes),
                    static_cast<unsigned long>(packed_flushes),
                    static_cast<unsigned long>(stripe_flushes));
      frame_counter = 0;
      fps_window_ms = now;
    }
  }
}

void microRadarSmokeSetState(const MicroRadarDisplayState& state) {
  display_state = state;
}

#ifndef MICRO_RADAR_SMOKE_AS_LIBRARY
void setup() {
  Serial.begin(115200);
  delay(1500);
  microRadarSmokeBegin();
}

void loop() {
  microRadarSmokeTick();
  delay(120);
}
#endif
