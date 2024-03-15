with import ( builtins.fetchTarball "https://github.com/NixOS/nixpkgs/archive/refs/tags/23.11.tar.gz" ) {
	overlays = [
		( import ( ( builtins.fetchGit {
			url = "git@github.com:lopsided98/nix-ros-overlay.git";
			ref = "develop";
			rev = "de1ea0a747635d6c0b33b7cfd99b75db6031d05d";
		} ) + "/overlay.nix" ) )
		( self: super: {
			gtsam = super.callPackage ./gtsam.nix {};
		} )
	];
};
callPackage ./package.nix {}
