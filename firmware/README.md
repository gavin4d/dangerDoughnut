# Danger Doughnut

Main firmware for Danger Doughnut

## Programming

To start a nix shell with esp-idf run `nix --experimental-features 'nix-command flakes' develop github:mirrexagon/nixpkgs-esp-dev#esp32-idf`.

Then run `idf.py build flash` to flash the driver esp.

## Dashboard

Danger Doughnut has been implemented with [Telometer](https://github.com/gagnonsilas/telometer) using a BLE backend.
To use the dashboard run `nix run` in `./components/telometer/`.
