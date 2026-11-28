# Sendspin playback

Configure Wi-Fi as usual, then set `SENDSPIN_SERVER_IP` to the Music Assistant server IPv4 address and `SENDSPIN_IDENTITY_KEY` to a persistent base64url-encoded 32-byte private key before building. Generate a separate key for each board, for example:

```sh
install -d -m 700 ~/.config/p4-dev-kit
test -s ~/.config/p4-dev-kit/sendspin-identity || (umask 077; python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > ~/.config/p4-dev-kit/sendspin-identity)
export SENDSPIN_SERVER_IP=192.168.1.10
export SENDSPIN_IDENTITY_KEY="$(cat ~/.config/p4-dev-kit/sendspin-identity)"
cargo build
```

Reuse the key on subsequent builds. It is embedded in the firmware, so keep the key and firmware private. Authorize the identity in Music Assistant and select the ESP32-P4 player. The client connects to port 8927 and supports 96 kHz, 16-bit stereo PCM, volume and mute. Continuous sample-rate correction is not implemented.

The codec uses a 24.576 MHz MCLK (256 Fs). The vendored es8311 0.1.1 driver adds the missing 24.576 MHz/96 kHz coefficient from Espressif’s esp_codec_dev driver. PCM and transport buffers are sized for the doubled data rate.

The P4 uses cached SRAM, unlike the S3 reference implementation. DMA descriptors must occupy separate cache lines so a CPU refill cannot overwrite ownership updates made by DMA. ESP-Hosted RPC replies are decoded into persistent buffers to avoid large nested stack copies. The SDIO receive buffer holds up to 64 KiB per batch; the network queues accommodate a full batch without waiting for RX space while TCP acknowledgements need sending.

A temporary TCP diagnostic received and acknowledged 8 MiB at about 999 KiB/s. The test endpoint is not included in the normal firmware. Playback still needs an audible continuity check after hardware changes.

Volume commands from Music Assistant are retained across Sendspin and Wi-Fi reconnections while the board remains powered. Volume commands received during clock synchronization are accepted as well. A board restart or power cycle starts at the default 25%; settings are not written to flash.
