{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    frc-nix.url = "github:frc4451/frc-nix";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      frc-nix,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ frc-nix.overlays.default ];
          config.allowUnfree = true;
        };
      in
      {
        devShells.default = pkgs.mkShell rec {
          buildInputs = with pkgs; [
            libglvnd
            libGL
            xorg.libXrandr
            xorg.libXinerama
            xorg.libXcursor
            xorg.libXi
            wayland
            clang
            wpilib.glass
            wpilib.sysid
            # jdk17
            stdenv.cc.cc.lib
            advantagescope
            wpilib.allwpilibSources
            wpilib.wpilib-utility
            (vscode-with-extensions.override {
              vscodeExtensions = with vscode-extensions; [
                wpilib.vscode-wpilib
                vscode-extensions.redhat.java
              ];
            })
          ];

          shellHook = ''

            export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${builtins.toString (pkgs.lib.makeLibraryPath buildInputs)}";
            export LIBCLANG_PATH="${pkgs.libclang.lib}/lib";
            export HALSIM_EXTENSIONS="$PWD/build/jni/release/libhalsim_gui.so";
            export JAVA_HOME="/home/churst/Downloads/wpilib-2026/WPILib_Linux-2026.2.1/jdk"
          '';
        };
      }
    );

  # packages.x86_64-linux.hello = nixpkgs.legacyPackages.x86_64-linux.hello;
  #
  # packages.x86_64-linux.default = self.packages.x86_64-linux.hello;
}
#    advantagescope
