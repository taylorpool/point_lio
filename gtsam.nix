{ lib, stdenv, cmake, ninja, eigen, boost, tbb, metis, fetchFromGitHub }:
stdenv.mkDerivation {
	pname = "gtsam";
	version = "4.2";

	src = fetchFromGitHub {
		owner = "borglab";
		repo = "gtsam";
		rev = "4.2.0";
		hash = "sha256-HjpGrHclpm2XsicZty/rX/RM/762wzmj4AAoEfni8es=";
	};

	nativeBuildInputs = [
		cmake
		ninja
	];

	propagatedBuildInputs = [
		eigen
		boost
		tbb
		metis
	];

	cmakeFlags = [
		"-DGTSAM_USE_SYSTEM_EIGEN=ON"
		"-DCMAKE_BUILD_TYPE=Release"
		"-DGTSAM_WITH_TBB=ON"
		"-DGTSAM_USE_SYSTEM_METIS=ON"
	];
}
