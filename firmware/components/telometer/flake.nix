{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    telometer.url = "github:gagnonsilas/telometer";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      telometer,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
        telometer-build = telometer.outputs.telometer-build;
      in
      with builtins;
      rec {

        devShells.default = pkgs.mkShell {
          name = "telomytest";
          packages = with pkgs; [
            packages.default
          ];

          shellHook = ''
            exec fish  
          '';
        };

        packages.default = (telometer-build.x86_64-linux { header = ./Packets.h; backend = ./BLEBackend.zig; main = ./main.zig;});

      }
    );
}

