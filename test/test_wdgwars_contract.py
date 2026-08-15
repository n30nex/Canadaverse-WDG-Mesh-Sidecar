import base64
import hashlib
import hmac
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
BOARD_SOURCE = (ROOT / "src" / "boards" / "BoardSupport.cpp").read_text(
    encoding="utf-8"
)
BOARD_HEADER = (ROOT / "src" / "boards" / "BoardSupport.h").read_text(
    encoding="utf-8"
)
PLATFORMIO = (ROOT / "platformio.ini").read_text(encoding="utf-8")
BUILD_SCRIPT = (ROOT / "scripts" / "meshcore_ed25519.py").read_text(
    encoding="utf-8"
)
NV_DISPLAY = (ROOT / "src" / "helpers" / "ui" / "NV3001BDisplay.cpp").read_text(
    encoding="utf-8"
)
CAPTURE_TOOL = (ROOT / "tools" / "capture_screen.py").read_text(encoding="utf-8")


def test_watchdogsgo_meshcore_contract_is_present():
    for field in (
        '"networks"',
        '"aircraft"',
        '"meshcore_nodes"',
        '"node_id"',
        '"node_type"',
        '"name"',
        '"lat"',
        '"lon"',
        '"rssi"',
        '"first_seen"',
        '"type"',
        '"MESHCORE"',
    ):
        assert field.replace('"', '\\"') in SOURCE


def test_hmac_envelope_matches_watchdogsgo_algorithm():
    payload = {
        "networks": [],
        "aircraft": [],
        "meshcore_nodes": [
            {
                "node_id": "01020304",
                "node_type": "Repeater",
                "name": "test-node",
                "lat": 43.6532,
                "lon": -79.3832,
                "rssi": -92.0,
                "first_seen": "2026-08-12T20:00:00",
                "type": "MESHCORE",
            }
        ],
    }
    key = "ab" * 32
    nonce = "0123456789abcdef"
    raw = json.dumps(payload, separators=(",", ":")).encode()
    data = base64.b64encode(raw).decode()
    expected = hmac.new(key.encode(), (nonce + data).encode(), hashlib.sha256).hexdigest()

    assert len(nonce) == 16
    assert len(expected) == 64
    assert expected == "e9cac2e307c34081feeacd6a69a83edf03ec1578ee57062f49f7eba14026e62d"
    assert "encodedCapacity + 1" in SOURCE


def test_no_history_or_insecure_tls_path():
    forbidden = (
        "--all-time",
        "wigle-latest",
        "api.wigle.net",
        "setInsecure",
        "meshcore_nodes.csv",
    )
    for token in forbidden:
        assert token not in SOURCE


def test_no_embedded_credential_shaped_literal():
    literals = re.findall(r'"([A-Fa-f0-9]{64})"', SOURCE)
    assert literals == []


def test_meshcore_sf7_receiver_uses_the_proven_preamble():
    assert "constexpr uint16_t kMeshCorePreambleSymbols = 32;" in SOURCE


def test_setup_uses_a_dedicated_ap_and_minimum_numeric_wpa2_pin():
    assert "WiFi.setAutoReconnect(false);" in SOURCE
    assert "WiFi.mode(WIFI_AP);" in SOURCE
    assert 'static constexpr char digits[] = "0123456789";' in SOURCE
    assert "for (size_t i = 0; i < 8; ++i)" in SOURCE


def test_wifi_reconnect_reports_only_safe_numeric_state():
    assert "info.wifi_sta_disconnected.reason" in SOURCE
    assert "WDGWars Wi-Fi disconnected: reason=%d" in SOURCE
    assert "WiFi.disconnect(false, false);" in SOURCE


def test_meshcore_upload_flushes_without_phone_but_never_replays_post():
    assert "constexpr uint32_t kMeshUploadQuietFlushMs = 15 * 1000UL;" in SOURCE
    assert "constexpr uint32_t kMeshUploadMaxBatchWaitMs = 60 * 1000UL;" in SOURCE
    assert (
        "meshClient.service(wdgConfigured && wdgAuthValid && !setupPortalActive);"
        in SOURCE
    )
    assert "MeshCore live batch window ended; WDGWars upload requested" in SOURCE
    assert "Once POST was attempted, never replay the batch automatically" in SOURCE
    assert "rememberAttemptedMeshBatch(sentCount);" in SOURCE
    assert "meshNodeAlreadyAttempted(nodeId)" in SOURCE


def test_repeater_poll_is_direct_bounded_and_never_invents_gps():
    assert "constexpr uint32_t kMeshPollCooldownMs = 30 * 60 * 1000UL;" in SOURCE
    assert "kMeshPollMaxAdvertHops = 1" in SOURCE
    assert "contact.out_path_len = pathLength;" in SOURCE
    assert "(pathHops - index - 1) * hashSize" in SOURCE
    assert 'sendLogin(*contact, "", estimatedTimeout)' in SOURCE
    assert "sendResult != MSG_SEND_SENT_DIRECT" in SOURCE
    assert "kMeshNeighbourRequestCount = 8" in SOURCE
    assert "kMeshNeighbourPrefixSize = 8" in SOURCE
    assert "kMeshPollRecordCount = 12" in SOURCE
    assert "bool allowPacketForward(const mesh::Packet *) override { return false; }" in SOURCE
    assert "if (!pollingAllowed || pollPhase != PollPhase::Idle" in SOURCE
    assert "meshClient.pollInProgress()" in SOURCE
    assert "Neighbour replies contain IDs/age/SNR only; GPS is never" in SOURCE
    assert "gps-from-signed-adverts" in SOURCE


def test_biscuit_pro_sidecar_never_duplicates_wifi_wardrive():
    assert "volatile bool meshStreamRequested = false;" in SOURCE
    assert 'startMeshCoreSidecar("wardrive")' in SOURCE
    assert 'startMeshCoreSidecar("wardriveall")' in SOURCE
    assert "MeshCore sidecar active; WiFi handled by Biscuit Pro" in SOURCE
    assert "BLE MeshCore status stream will resume after notifications" in SOURCE
    assert "meshStreamRequested = false;" in SOURCE
    assert "WiFi.scanNetworks" not in SOURCE
    assert "DATA:AP:" not in SOURCE

    start = SOURCE.index("void startMeshCoreSidecar")
    stop = SOURCE.index("void stopWardrive", start)
    assert "meshUploadCount = 0" not in SOURCE[start:stop]


def test_all_three_exact_board_targets_are_pinned():
    assert "default_envs = rcc6_wdg_mesh, heltec_v3_wdg_mesh, heltec_v4_wdg_mesh" in PLATFORMIO
    assert "[env:rcc6_wdg_mesh]" in PLATFORMIO
    assert "[env:heltec_v3_wdg_mesh]" in PLATFORMIO
    assert "[env:heltec_v4_wdg_mesh]" in PLATFORMIO
    assert "platformio/espressif32@6.11.0" in PLATFORMIO
    assert "MeshCore.git#727fc0512ce08bfd7b499e46daa7fca6eeec730d" in PLATFORMIO
    assert "h2zero/NimBLE-Arduino @ 2.5.1" in PLATFORMIO
    assert "jgromes/RadioLib @ 7.7.1" in PLATFORMIO


def test_heltec_v3_uses_the_wired_sx1262_reset_pin():
    v3_start = BOARD_HEADER.index("#elif defined(WDG_BOARD_HELTEC_V3)")
    v4_start = BOARD_HEADER.index("#elif defined(WDG_BOARD_HELTEC_V4)", v3_start)
    assert "constexpr int kLoRaReset = 12;" in BOARD_HEADER[v3_start:v4_start]


def test_v4_front_end_uses_upstream_power_and_rx_patch_contract():
    for token in (
        "constexpr int kFemPowerPin = 7;",
        "constexpr int kFemSharedEnablePin = 2;",
        "constexpr int kFemGc1109TxPin = 46;",
        "constexpr int kFemKct8103TxPin = 5;",
        "radio.readRegister(0x08B5",
        "radio.writeRegister(0x08B5",
    ):
        assert token in BOARD_SOURCE
    assert "constexpr int8_t kRadioTxPowerDbm = 10;" in BOARD_HEADER
    assert "constexpr int8_t kEffectiveTxPowerDbm = 22;" in BOARD_HEADER
    assert "-I variants/heltec_v4" in PLATFORMIO
    assert "-D RADIOLIB_LOW_LEVEL=1" in PLATFORMIO
    v3_start = BOARD_SOURCE.index("#if defined(WDG_BOARD_HELTEC_V3)")
    v4_start = BOARD_SOURCE.index("#elif defined(WDG_BOARD_HELTEC_V4)", v3_start)
    board_end = BOARD_SOURCE.index("#endif", v4_start)
    assert "digitalWrite(36, LOW);" in BOARD_SOURCE[v3_start:v4_start]
    assert "digitalWrite(36, HIGH);" in BOARD_SOURCE[v4_start:board_end]


def test_public_setup_has_physical_button_and_captive_dns():
    assert "constexpr uint32_t kSetupButtonHoldMs = 3 * 1000UL;" in SOURCE
    assert "digitalRead(wdg_board::kUserButtonPin) == LOW" in SOURCE
    assert 'configDns.start(53, "*", WiFi.softAPIP())' in SOURCE
    assert "configDns.processNextRequest();" in SOURCE


def test_commands_and_credentials_are_not_printed_verbatim():
    assert 'Serial.println(command)' not in SOURCE
    assert 'Serial.print("RX ")' not in SOURCE
    assert 'Serial.printf("RX command=%s bytes=%u' in SOURCE
    assert "wdgWifiPassword.c_str()" not in SOURCE.replace(
        "WiFi.begin(wdgWifiSsid.c_str(), wdgWifiPassword.c_str())", ""
    )
    assert "wdgApiKey.c_str()" in SOURCE  # Used only as HMAC input/header value.


def test_only_the_pinned_nimble_stack_is_selected():
    assert "h2zero/NimBLE-Arduino @ 2.5.1" in PLATFORMIO
    assert "ESP32 BLE Arduino" in PLATFORMIO


def test_meshcore_build_excludes_unused_board_and_sensor_helpers():
    for source in (
        "AdvertDataHelpers.cpp",
        "BaseChatMesh.cpp",
        "StaticPoolPacketManager.cpp",
        "TxtDataHelpers.cpp",
    ):
        assert source in BUILD_SCRIPT
    assert '+<helpers/*.cpp>' not in BUILD_SCRIPT
    for ignored in ("RTClib", "Melopero RV3028", "CayenneLPP"):
        assert ignored in PLATFORMIO


def test_screenshot_protocol_is_explicit_crc_checked_and_dormant_by_default():
    assert "volatile bool wdg_screenshot_requested = false;" in SOURCE
    assert 'command == "CMD:screenshot:"' in SOURCE
    assert "if (!wdg_screenshot_requested || framebuffer == nullptr) return;" in NV_DISPLAY
    assert "if (!wdg_screenshot_requested) return;" in BUILD_SCRIPT
    assert "binascii.crc32(payload)" in CAPTURE_TOOL
    assert 'port.write(b"CMD:screenshot:\\n")' in CAPTURE_TOOL
    assert 'isScreenshot ? "********" : setupPassword.c_str()' in SOURCE
