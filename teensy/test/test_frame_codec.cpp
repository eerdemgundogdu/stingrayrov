// frame_codec icin host birim testleri.
//
//   cmake -S teensy/test -B build/test && cmake --build build/test && ctest --test-dir build/test

#include "minitest.h"
#include "frame_codec.h"
#include "dsp_filter.h"

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace {

const uint16_t kNeutral[FRAME_MOTOR_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};
const uint16_t kMixed[FRAME_MOTOR_COUNT]   = {1100, 1900, 1500, 1234, 1777, 1600};

// Jetson tarafi (utils.crc8 + serial_bridge) ile ayni cerceve; ayni bayt dizisi
// jetson/test/test_protocol.py icinde de dogrulaniyor.
const uint8_t kGoldenNeutral[FRAME_CMD_LEN] = {
    0xAA, 0x55, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC,
    0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x09};

struct FeedResult {
    int frames = 0;
    uint16_t last[FRAME_MOTOR_COUNT] = {};
};

FeedResult feed_all(CmdFrameParser &p, const std::vector<uint8_t> &bytes)
{
    FeedResult r;
    uint16_t pwm[FRAME_MOTOR_COUNT];
    for (uint8_t b : bytes) {
        if (cmd_parser_feed(&p, b, pwm) == FRAME_OK) {
            r.frames++;
            std::memcpy(r.last, pwm, sizeof(pwm));
        }
    }
    return r;
}

std::vector<uint8_t> encode(const uint16_t pwm[FRAME_MOTOR_COUNT])
{
    uint8_t f[FRAME_CMD_LEN];
    cmd_frame_encode(pwm, f);
    return std::vector<uint8_t>(f, f + FRAME_CMD_LEN);
}

void append(std::vector<uint8_t> &dst, const std::vector<uint8_t> &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

} // namespace

TEST(crc8_matches_smbus_check_value)
{
    const char *s = "123456789";
    CHECK_EQ(frame_crc8(reinterpret_cast<const uint8_t *>(s), 9), 0xF4);
    CHECK_EQ(frame_crc8(nullptr, 0), 0x00);
}

TEST(encoder_matches_golden_frame_shared_with_python)
{
    std::vector<uint8_t> f = encode(kNeutral);
    CHECK(std::memcmp(f.data(), kGoldenNeutral, FRAME_CMD_LEN) == 0);
}

TEST(round_trip_single_frame)
{
    CmdFrameParser p;
    cmd_parser_init(&p);
    FeedResult r = feed_all(p, encode(kMixed));
    CHECK_EQ(r.frames, 1);
    for (int i = 0; i < FRAME_MOTOR_COUNT; i++) CHECK_EQ(r.last[i], kMixed[i]);
    CHECK_EQ(p.crc_errors, 0);
}

TEST(back_to_back_frames)
{
    CmdFrameParser p;
    cmd_parser_init(&p);
    std::vector<uint8_t> s;
    for (int i = 0; i < 50; i++) append(s, encode(i % 2 ? kMixed : kNeutral));
    FeedResult r = feed_all(p, s);
    CHECK_EQ(r.frames, 50);
    CHECK_EQ(r.last[1], kMixed[1]);
    CHECK_EQ(p.bytes_dropped, 0);
}

TEST(leading_garbage_is_skipped)
{
    CmdFrameParser p;
    cmd_parser_init(&p);
    std::vector<uint8_t> s = {0x00, 0xFF, 0xAA, 0x00, 0xAA, 0xAA, 0x13};
    append(s, encode(kMixed));
    FeedResult r = feed_all(p, s);
    CHECK_EQ(r.frames, 1);
    CHECK_EQ(r.last[0], kMixed[0]);
}

TEST(repeated_header_byte_does_not_lose_sync)
{
    // 0xAA 0xAA 0x55 ... : ilk 0xAA atilmali, ikinci 0xAA baslik olmali.
    CmdFrameParser p;
    cmd_parser_init(&p);
    std::vector<uint8_t> s = {0xAA};
    append(s, encode(kMixed));
    FeedResult r = feed_all(p, s);
    CHECK_EQ(r.frames, 1);
}

TEST(corrupted_frame_is_rejected)
{
    CmdFrameParser p;
    cmd_parser_init(&p);
    std::vector<uint8_t> s = encode(kMixed);
    s[5] ^= 0x01;
    FeedResult r = feed_all(p, s);
    CHECK_EQ(r.frames, 0);
    CHECK(p.crc_errors >= 1);
}

TEST(valid_frame_inside_truncated_frame_is_recovered)
{
    // Eski uygulamadaki hata: bozuk cercevenin 15 baytinin tamami atiliyordu.
    // Burada kesik bir cercevenin hemen ardindan gelen gecerli cerceve,
    // kesik cercevenin "icine" dusuyor ve yine de yakalanmali.
    CmdFrameParser p;
    cmd_parser_init(&p);
    std::vector<uint8_t> cut = encode(kNeutral);
    cut.resize(6);                      // baglanti koptu, 6 bayt geldi
    std::vector<uint8_t> s = cut;
    append(s, encode(kMixed));
    FeedResult r = feed_all(p, s);
    CHECK_EQ(r.frames, 1);
    for (int i = 0; i < FRAME_MOTOR_COUNT; i++) CHECK_EQ(r.last[i], kMixed[i]);
}

TEST(stream_survives_random_noise)
{
    // Gecerli cerceveler arasina rastgele gurultu: hicbir sahte cerceve
    // kabul edilmemeli ve gurultuden sonra gelen cerceveler kaybolmamali.
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> byte(0, 255), gap(0, 20);
    CmdFrameParser p;
    cmd_parser_init(&p);

    int expected = 0, got = 0, bogus = 0;
    uint16_t pwm[FRAME_MOTOR_COUNT];
    for (int k = 0; k < 2000; k++) {
        const int n = gap(rng);
        for (int i = 0; i < n; i++) {
            if (cmd_parser_feed(&p, (uint8_t)byte(rng), pwm) == FRAME_OK) bogus++;
        }
        // Gurultu bir cercevenin ortasinda biterse sonraki gecerli cercevenin
        // ilk baytlari o yarim cerceveye eklenir; parser yine de toparlanmali.
        for (uint8_t b : encode(kMixed)) {
            if (cmd_parser_feed(&p, b, pwm) == FRAME_OK) {
                if (std::memcmp(pwm, kMixed, sizeof(pwm)) == 0) got++;
                else bogus++;
            }
        }
        expected++;
    }
    // CRC-8 ile rastgele bir cercevenin gecme ihtimali ~1/256 * baslik
    // olasiligi; 2000 tekrar icin pratikte sifir.
    CHECK_EQ(bogus, 0);
    CHECK_EQ(got, expected);
}

TEST(clamp_pwm)
{
    CHECK_EQ(frame_clamp_pwm(900, 1100, 1900), 1100);
    CHECK_EQ(frame_clamp_pwm(2500, 1100, 1900), 1900);
    CHECK_EQ(frame_clamp_pwm(1500, 1100, 1900), 1500);
}

TEST(slew_pwm_deadband_and_ramp_limits)
{
    // Notr cevresinde +-12 us olu bolge 1500'e yuvarlanmali
    CHECK_EQ(frame_slew_pwm(1500, 1511, 20, 12), 1500);
    CHECK_EQ(frame_slew_pwm(1500, 1489, 20, 12), 1500);
    CHECK_EQ(frame_slew_pwm(1500, 1512, 20, 12), 1512);

    // Tam ileri (1900) hedefe 20 us/ms adimlarla 20 adimda ulasmali
    uint16_t cur = 1500;
    for (int i = 0; i < 19; i++) {
        cur = frame_slew_pwm(cur, 1900, 20, 12);
        CHECK_EQ(cur, 1500 + (i + 1) * 20);
    }
    cur = frame_slew_pwm(cur, 1900, 20, 12);
    CHECK_EQ(cur, 1900);

    // Tam ileriden (1900) tam geriye (1100) ani geciste 1 adimda en fazla 20 us duser
    CHECK_EQ(frame_slew_pwm(1900, 1100, 20, 12), 1880);

    // Sinir disi hedef once [1100, 1900] araligina cekilir
    CHECK_EQ(frame_slew_pwm(1890, 2500, 20, 12), 1900);
    CHECK_EQ(frame_slew_pwm(1110, 500, 20, 12), 1100);
}

TEST(sensor_frame_round_trip_with_negative_angles)
{
    SensorFrame in = {35999, -1234, 4500, 2750, 1, 2310};
    uint8_t buf[FRAME_SENSOR_LEN];
    sensor_frame_encode(&in, buf);
    CHECK_EQ(buf[0], 0xBB);
    CHECK_EQ(buf[1], 0x66);
    CHECK_EQ(buf[13], 0);
    CHECK_EQ(buf[15], 0);

    SensorFrame out;
    CHECK(sensor_frame_decode(buf, &out));
    CHECK_EQ(out.yaw_cdeg, in.yaw_cdeg);
    CHECK_EQ(out.pitch_cdeg, in.pitch_cdeg);
    CHECK_EQ(out.roll_cdeg, in.roll_cdeg);
    CHECK_EQ(out.depth_mm, in.depth_mm);
    CHECK_EQ(out.leak, in.leak);
    CHECK_EQ(out.battery_cv, in.battery_cv);

    buf[9] ^= 0x80;
    CHECK(!sensor_frame_decode(buf, &out));
}

TEST(biquad_notch_rejects_thruster_resonance_passes_maneuver)
{
    // 1 kHz IMU orneklemesinde 120 Hz itici govde vibrasyonu >25 dB bastirilmali,
    // 2 Hz arac yaw manevrasi bozulmadan gecmeli.
    BiquadFilter notch{};
    BiquadFilter lpf{};
    notch.configure_notch(1000.0f, 120.0f, 4.0f);
    lpf.configure_lowpass(1000.0f, 35.0f, 0.70710678f);

    float vib_rms_acc = 0.0f;
    float low_rms_acc = 0.0f;
    const int n_samples = 1000;
    for (int i = 0; i < n_samples; ++i) {
        float t = static_cast<float>(i) / 1000.0f;
        float x_vib = std::sin(2.0f * 3.14159265f * 120.0f * t);
        float y_vib = lpf.step(notch.step(x_vib));
        if (i >= 200) {
            vib_rms_acc += y_vib * y_vib;
        }
    }
    notch.reset();
    lpf.reset();
    for (int i = 0; i < n_samples; ++i) {
        float t = static_cast<float>(i) / 1000.0f;
        float x_low = std::sin(2.0f * 3.14159265f * 2.0f * t);
        float y_low = lpf.step(notch.step(x_low));
        if (i >= 200) {
            low_rms_acc += y_low * y_low;
        }
    }
    float vib_rms = std::sqrt(vib_rms_acc / 800.0f);
    float low_rms = std::sqrt(low_rms_acc / 800.0f);
    CHECK(vib_rms < 0.015f);
    CHECK(low_rms > 0.68f);
}

TEST(median5_rejects_adc_glitch_spikes)
{
    Median5Filter mf{};
    mf.init(0.5f);
    for (int i = 0; i < 10; ++i) {
        mf.step(1.50f);
    }
    // Tek orneklik 15.0 m ADC sicramasi (EMI / ESC anahtarlama dikeni) reddedilmeli
    float out_spike = mf.step(15.0f);
    CHECK(std::fabs(out_spike - 1.50f) < 1e-4f);
}

TEST_MAIN()
