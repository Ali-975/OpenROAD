///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2023, Google LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define BOOST_TEST_MODULE CFileUtilsTest

#ifdef HAS_BOOST_UNIT_TEST_LIBRARY
// Shared library version
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#else
// Header only version
#include <boost/test/included/unit_test.hpp>
#endif

#include <filesystem>

#include "utl/CFileUtils.h"
#include "utl/ScopedTemporaryFile.h"
namespace utl {

BOOST_AUTO_TEST_CASE(read_all_of_empty_file)
{
  Logger logger;
  ScopedTemporaryFile stf(&logger);
  std::string contents = GetContents(stf.file(), &logger);
  BOOST_TEST(contents.empty());
}

// Writes then reads 4B of data.
BOOST_AUTO_TEST_CASE(read_all_of_written_file_seek_required)
{
  Logger logger;
  ScopedTemporaryFile stf(&logger);

  const std::vector<uint8_t> kTestData = {0x01, 0x02, 0x03, 0x04};
  WriteAll(stf.file(), kTestData, &logger);

  std::string contents = GetContents(stf.file(), &logger);
  BOOST_TEST(contents.size() == kTestData.size());
  for (size_t i = 0; i < contents.size(); ++i) {
    BOOST_TEST(static_cast<uint8_t>(contents.at(i)) == kTestData.at(i));
  }
}

// Writes then reads 1024B of data.
BOOST_AUTO_TEST_CASE(read_all_of_file_exactly_1024B)
{
  Logger logger;
  ScopedTemporaryFile stf(&logger);

  std::vector<uint8_t> test_data(1024);
  std::iota(test_data.begin(), test_data.end(), 0);

  WriteAll(stf.file(), test_data, &logger);

  std::string contents = GetContents(stf.file(), &logger);
  BOOST_TEST(contents.size() == test_data.size());
  for (size_t i = 0; i < contents.size(); ++i) {
    BOOST_TEST(static_cast<uint8_t>(contents.at(i)) == test_data.at(i));
  }
}

// Writes then reads 1025B of data (whitebox test, we know internally the read
// buffer size is 1024B so this causes two chunks of read).
BOOST_AUTO_TEST_CASE(read_all_of_file_exactly_1025B)
{
  Logger logger;
  ScopedTemporaryFile stf(&logger);

  std::vector<uint8_t> test_data(1025);
  std::iota(test_data.begin(), test_data.end(), 0);

  WriteAll(stf.file(), test_data, &logger);

  std::string contents = GetContents(stf.file(), &logger);
  BOOST_TEST(contents.size() == test_data.size());
  for (size_t i = 0; i < contents.size(); ++i) {
    BOOST_TEST(static_cast<uint8_t>(contents.at(i)) == test_data.at(i));
  }
}

// Add new tests for StreamHandler
BOOST_AUTO_TEST_CASE(stream_handler_write_and_read)
{
  const char* filename = "test_write_and_read.txt";
  const std::vector<uint8_t> kTestData = {0x01, 0x02, 0x03, 0x04};

  {
    StreamHandler sh(filename);
    std::ofstream& os = sh.getStream();
    os.write(reinterpret_cast<const char*>(kTestData.data()), kTestData.size());
  }

  std::ifstream is(filename, std::ios_base::binary);
  std::string contents((std::istreambuf_iterator<char>(is)),
                       std::istreambuf_iterator<char>());
  BOOST_TEST(contents.size() == kTestData.size());
  for (size_t i = 0; i < contents.size(); ++i) {
    BOOST_TEST(static_cast<uint8_t>(contents.at(i)) == kTestData.at(i));
  }
}

BOOST_AUTO_TEST_CASE(stream_handler_temp_file_handling)
{
  const char* filename = "test_temp_file_handling.txt";
  std::string tmp_filename = std::string(filename) + ".tmp";

  // Check that the temp file is created
  {
    StreamHandler sh(filename);
    BOOST_TEST(std::filesystem::exists(tmp_filename));
  }

  // Check that the temp file is renamed to the original filename
  BOOST_TEST(!std::filesystem::exists(tmp_filename));
  BOOST_TEST(std::filesystem::exists(filename));
}

BOOST_AUTO_TEST_CASE(stream_handler_exception_handling)
{
  const char* filename = "test_exception_handling.txt";

  // Ensure the temporary file is handled correctly if an exception occurs
  try {
    StreamHandler sh(filename);
    throw std::runtime_error("Simulated exception");
  } catch (...) {
    std::string tmp_filename = std::string(filename) + ".tmp";
    BOOST_TEST(!std::filesystem::exists(
        tmp_filename));  // Ensure temporary file is cleaned up
    BOOST_TEST(std::filesystem::exists(
        filename));  // Original file should exist either
  }
}

// Add new tests for FileHandler
BOOST_AUTO_TEST_CASE(file_handler_write_and_read)
{
  const char* filename = "test_write_and_read_file.txt";
  const std::vector<uint8_t> kTestData = {0x01, 0x02, 0x03, 0x04};
  {
    FileHandler fh(filename, true);  // binary mode
    FILE* file = fh.getFile();
    fwrite(kTestData.data(), sizeof(uint8_t), kTestData.size(), file);
  }

  std::ifstream is(filename, std::ios_base::binary);
  std::string contents((std::istreambuf_iterator<char>(is)),
                       std::istreambuf_iterator<char>());
  BOOST_TEST(contents.size() == kTestData.size());
  for (size_t i = 0; i < contents.size(); ++i) {
    BOOST_TEST(static_cast<uint8_t>(contents.at(i)) == kTestData.at(i));
  }
}

BOOST_AUTO_TEST_CASE(file_handler_temp_file_handling)
{
  const char* filename = "test_temp_file_handling_file.txt";
  std::string tmp_filename = std::string(filename) + ".tmp";

  // Check that the temp file is created
  {
    FileHandler fh(filename, true);  // binary mode
    BOOST_TEST(std::filesystem::exists(tmp_filename));
  }

  // Check that the temp file is renamed to the original filename
  BOOST_TEST(!std::filesystem::exists(tmp_filename));
  BOOST_TEST(std::filesystem::exists(filename));
}

BOOST_AUTO_TEST_CASE(file_handler_exception_handling)
{
  const char* filename = "test_exception_handling_file.txt";

  // Ensure the temporary file is handled correctly if an exception occurs
  try {
    FileHandler fh(filename, true);  // binary mode
    throw std::runtime_error("Simulated exception");
  } catch (...) {
    std::string tmp_filename = std::string(filename) + ".tmp";
    BOOST_TEST(!std::filesystem::exists(
        tmp_filename));  // Ensure temporary file is cleaned up
    BOOST_TEST(
        std::filesystem::exists(filename));  // Original file should exist
  }
}

}  // namespace utl
