
let
  nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-23.11";
  pkgs = import nixpkgs { config = {}; overlays = []; };

  my-python-packages = ps: with ps; [
    pyusb
    pycrypto
    pycairo
    pydbus
    pygobject3
    setuptools
  ];
in
pkgs.mkShell {
  packages = with pkgs; [
    (pkgs.python312.withPackages my-python-packages)
    git
    wget
    flex
    bison
    gperf
    #python3-venv
    cmake 
    ccache 
    dfu-util
    libusb1
  ];
}
