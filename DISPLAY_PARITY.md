# Display baseline and performance

Reference: c-demo, ESP-IDF v5.5.5, waveshare/esp_lcd_hx8394 2.1.0.

Rust matches the reference reset, DCS command bytes/order/delays, RGB565 format, 700 Mbps two-lane link, requested 58 MHz pixel clock, and horizontal (20/20/40) and vertical (4/10/24) sync/back/front porches. Rust also waits 1 second after the backlight controller sequence, as the C constructor does.

Measured Rust VSYNC: 120 intervals in 2,182,436 us = 54.984 Hz. Hardware pattern is disabled; VDMA reads the PSRAM RGB565 framebuffer. Serial reports the sticky bridge underrun flag every 5 seconds.

This is a working reference profile, not a measured rendering FPS benchmark. One RGB565 frame is 1,843,200 bytes; scanout reads about 101.35 MB/s at the measured refresh rate. The panel is advertised up to 60 Hz. Reaching that requires a separately validated timing profile; raising the requested clock alone does not establish 60 Hz because esp-hal uses integer clock division and porch compensation.

Remaining HAL differences: ESP-IDF permits LP during active vertical video and enables frame ACK; esp-hal deliberately disables both. Preserve these HAL choices pending targeted validation. The DMA implementations also differ. Full equivalence of PHY register values and rendered frame presentation has not been established.

The static framebuffer is never modified during scanout. For animation, validate VSYNC synchronized buffer presentation and actual completed frames before claiming rendering FPS or tear-free double buffering.

## Framebuffer investigation

The C demo has now been verified visually with PSRAM software bars matching Rust, and is straight and stable. Rust skew remained after implementing full-frame DMA completion/restart and after reducing PSRAM to 125 MHz. A 200 MHz HAL PSRAM preset stalls initialization on this board. Framebuffer alignment has been increased to 128 bytes.

The local esp-hal patch now writes back L1 before L2, as ESP-IDF cache_ll_writeback_addr does. This is awaiting visual confirmation; do not treat this investigation as a proven fix yet. The local DMA patch is also experimental. It configures a single final descriptor, handles full-transfer completion, and starts DMA after interrupt setup.

## Confirmed working framebuffer configuration

User confirmed straight and stable Rust PSRAM framebuffer bars after enabling both `lp_vact_en` and `frame_bta_ack_en`, matching ESP-IDF. These were the final changes relative to the still-skewed build; their individual necessity has not been isolated. Current measured VSYNC is about 55.175 Hz (120 intervals in 2,174,865 us, including startup register-print overhead effects). Rendering FPS is not measured.

The current local HAL patch also includes single-final-descriptor DMA restart and sequential L1/L2 writeback. Those changes alone did not fix skew. PSRAM is currently 125 MHz and framebuffer alignment is 128 bytes. The 200 MHz HAL preset stalled initialization; the C driver uses a different PSRAM setup. Do not claim complete PSRAM/PHY/performance parity or 60 FPS from this result.

Earlier statements about preserving disabled frame ACK and active-video LP are superseded by this hardware-confirmed result. Keep the working C demo as a regression reference.
