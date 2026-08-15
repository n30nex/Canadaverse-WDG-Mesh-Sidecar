# Heltec V4 hardware validation

Candidate: `v1.0.0-rc.2`

Application SHA-256: `2191fac5382c66e7b8269a41416e3a08f4b7d9081d6f036d8ee2226e01e00ba3`

The candidate was installed app-only at `0x10000` on a physical Heltec WiFi
LoRa 32 V4/V4.3. The original 16 MB flash was backed up before the first write.
The final application read-back matched the candidate digest.

Validated together on the final candidate:

- V4 OLED power enabled with active-low GPIO 36, matching Heltec's V4 examples.
- SSD1306 initialized at 128 by 64 and returned a CRC-checked framebuffer.
- Canada MeshCore radio initialized at 910.525 MHz, 62.5 kHz, SF7, CR 4/5,
  with an effective 22 dBm transmit profile.
- USB provisioning accepted protected hotspot and WDGWars settings without
  echoing them, rebooted, and reached `WDGWars status: READY`.
- A controlled, signed MeshCore advert with a signed location was received over
  LoRa. WDGWars accepted one live batch; the display then showed `+1 Q0`.
- Three consecutive BLE rounds passed advertisement discovery, connection,
  service and identity reads, command write, READY notification, disconnect,
  re-advertisement, and reconnect.
- All 19 contract tests passed, and RCC6, Heltec V3, and Heltec V4 builds
  completed successfully.

The board was bolted inside an enclosure, so direct photography of the OLED was
not possible. [`heltec-v4-wdg-after-mesh.png`](heltec-v4-wdg-after-mesh.png) is
the release evidence captured from the exact framebuffer sent to the physical
display driver; it contains no credentials or location data.
