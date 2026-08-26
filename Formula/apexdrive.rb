class Apexdrive < Formula
  desc "Universal High-Performance Robotics Actuator & Motor Control Engine"
  homepage "https://github.com/Himan-D/apexdrive"
  url "https://github.com/Himan-D/apexdrive/archive/refs/tags/v1.0.0.tar.gz"
  sha256 "PLACEHOLDER_SHA256"
  license "Apache-2.0"
  head "https://github.com/Himan-D/apexdrive.git", branch: "main"

  depends_on "cmake" => :build

  def install
    system "cmake", "-S", ".", "-B", "build", *std_cmake_args
    system "cmake", "--build", "build"
    system "cmake", "--install", "build"
  end

  test do
    output = shell_output("#{bin}/apexdrive version")
    assert_match "ApexDrive System Version", output
  end
end
