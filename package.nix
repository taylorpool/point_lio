{ lib, stdenv, rosPackages, eigen, gtsam, cmake, ninja }:
stdenv.mkDerivation {
	pname = "point_lio";
	version = "0.1.0";

	src = ./.;

	nativeBuildInputs = [
		cmake
		ninja
	];

	propagatedBuildInputs = [
		rosPackages.noetic.sensor-msgs
		rosPackages.noetic.velodyne-msgs
		rosPackages.noetic.nav-msgs
		rosPackages.noetic.geometry-msgs
		rosPackages.noetic.roscpp
		rosPackages.noetic.tf2
		rosPackages.noetic.tf2-ros
		eigen
		gtsam
	];
}
