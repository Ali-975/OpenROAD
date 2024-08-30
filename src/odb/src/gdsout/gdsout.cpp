///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2019, Nefelus Inc
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

#include "odb/gdsout.h"

#include <iostream>

#include "../db/dbGDSBoundary.h"
#include "../db/dbGDSElement.h"
#include "../db/dbGDSLib.h"
#include "../db/dbGDSPath.h"
#include "../db/dbGDSSRef.h"
#include "../db/dbGDSStructure.h"
#include "../db/dbGDSText.h"

namespace odb {

GDSWriter::GDSWriter() : _lib(nullptr)
{
}

GDSWriter::~GDSWriter()
{
  if (_file.is_open()) {
    _file.close();
  }
}

void GDSWriter::writeGDSFile(dbGDSLib* lib, const std::string& filename)
{
  _lib = lib;
  _file.open(filename, std::ios::binary);
  if (!_file.is_open()) {
    throw std::runtime_error("Unable to open file");
  }
  writeLibrary();
  _file.close();
  _lib = nullptr;
}

void GDSWriter::calculateRecordSize(record_t& record)
{
  record.length = 4;
  switch (record.dataType) {
    case DataType::REAL_8:
      record.length += record.data64.size() * 8;
      break;
    case DataType::INT_4:
      record.length += record.data32.size() * 4;
      break;
    case DataType::INT_2:
      record.length += record.data16.size() * 2;
      break;
    case DataType::ASCII_STRING:
    case DataType::BIT_ARRAY:
      record.length += record.data8.size();
      break;
    case DataType::NO_DATA:
      break;
    default:
      throw std::runtime_error("Invalid data type encountered");
  }
}

void GDSWriter::writeReal8(double value)
{
  uint64_t real8 = htobe64(double_to_real8(value));
  _file.write(reinterpret_cast<char*>(&real8), sizeof(uint64_t));
}

void GDSWriter::writeInt32(int32_t value)
{
  int32_t network_order_value = htobe32(value);
  _file.write(reinterpret_cast<char*>(&network_order_value), sizeof(int32_t));
}

void GDSWriter::writeInt16(int16_t value)
{
  int16_t network_order_value = htobe16(value);
  _file.write(reinterpret_cast<char*>(&network_order_value), sizeof(int16_t));
}

void GDSWriter::writeInt8(int8_t value)
{
  _file.write(reinterpret_cast<char*>(&value), sizeof(int8_t));
}

void GDSWriter::writeRecord(record_t& record)
{
  calculateRecordSize(record);
  writeInt16(record.length);
  writeInt8(fromRecordType(record.type));
  writeInt8(fromDataType(record.dataType));

  switch (record.dataType) {
    case DataType::REAL_8:
      for (auto& data : record.data64) {
        writeReal8(data);
      }
      break;
    case DataType::INT_4:
      for (auto& data : record.data32) {
        writeInt32(data);
      }
      break;
    case DataType::INT_2:
      for (auto& data : record.data16) {
        writeInt16(data);
      }
      break;
    case DataType::ASCII_STRING:
    case DataType::BIT_ARRAY:
      _file.write(record.data8.c_str(), record.data8.size());
      break;
    default:
      break;
  }
}

void GDSWriter::writeLibrary()
{
  record_t header_record;
  header_record.type = RecordType::HEADER;
  header_record.dataType = DataType::INT_2;
  header_record.data16 = {600};
  writeRecord(header_record);

  record_t lib_begin_record;
  lib_begin_record.type = RecordType::BGNLIB;
  lib_begin_record.dataType = DataType::INT_2;
  
  std::time_t current_time = std::time(nullptr);
  std::tm* local_time = std::localtime(&current_time);
  lib_begin_record.data16 = {static_cast<int16_t>(local_time->tm_year),
                             static_cast<int16_t>(local_time->tm_mon),
                             static_cast<int16_t>(local_time->tm_mday),
                             static_cast<int16_t>(local_time->tm_hour),
                             static_cast<int16_t>(local_time->tm_min),
                             static_cast<int16_t>(local_time->tm_sec),
                             static_cast<int16_t>(local_time->tm_year),
                             static_cast<int16_t>(local_time->tm_mon),
                             static_cast<int16_t>(local_time->tm_mday),
                             static_cast<int16_t>(local_time->tm_hour),
                             static_cast<int16_t>(local_time->tm_min),
                             static_cast<int16_t>(local_time->tm_sec)};
  writeRecord(lib_begin_record);

  record_t lib_name_record;
  lib_name_record.type = RecordType::LIBNAME;
  lib_name_record.dataType = DataType::ASCII_STRING;
  lib_name_record.data8 = _lib->getLibname();
  writeRecord(lib_name_record);

  record_t units_record;
  units_record.type = RecordType::UNITS;
  units_record.dataType = DataType::REAL_8;
  auto units = _lib->getUnits();
  units_record.data64 = {units.first, units.second};
  writeRecord(units_record);

  for (auto structure : _lib->getGDSStructures()) {
    writeStructure(structure);
  }

  record_t end_lib_record;
  end_lib_record.type = RecordType::ENDLIB;
  end_lib_record.dataType = DataType::NO_DATA;
  writeRecord(end_lib_record);
}

void GDSWriter::writeStructure(dbGDSStructure* structure)
{
  record_t struct_begin_record;
  struct_begin_record.type = RecordType::BGNSTR;
  struct_begin_record.dataType = DataType::INT_2;
  
  std::time_t current_time = std::time(nullptr);
  std::tm* local_time = std::localtime(&current_time);
  struct_begin_record.data16 = {static_cast<int16_t>(local_time->tm_year),
                                static_cast<int16_t>(local_time->tm_mon),
                                static_cast<int16_t>(local_time->tm_mday),
                                static_cast<int16_t>(local_time->tm_hour),
                                static_cast<int16_t>(local_time->tm_min),
                                static_cast<int16_t>(local_time->tm_sec),
                                static_cast<int16_t>(local_time->tm_year),
                                static_cast<int16_t>(local_time->tm_mon),
                                static_cast<int16_t>(local_time->tm_mday),
                                static_cast<int16_t>(local_time->tm_hour),
                                static_cast<int16_t>(local_time->tm_min),
                                static_cast<int16_t>(local_time->tm_sec)};
  writeRecord(struct_begin_record);

  record_t struct_name_record;
  struct_name_record.type = RecordType::STRNAME;
  struct_name_record.dataType = DataType::ASCII_STRING;
  struct_name_record.data8 = structure->getName();
  writeRecord(struct_name_record);

  for (auto element : ((_dbGDSStructure*)structure)->_elements) {
    writeElement(static_cast<dbGDSElement*>(element));
  }

  record_t end_struct_record;
  end_struct_record.type = RecordType::ENDSTR;
  end_struct_record.dataType = DataType::NO_DATA;
  writeRecord(end_struct_record);
}

void GDSWriter::writeElement(dbGDSElement* element)
{
  _dbGDSElement* internal_element = static_cast<_dbGDSElement*>(element);
  
  if (dynamic_cast<_dbGDSBoundary*>(internal_element) != nullptr) {
    writeBoundary(static_cast<dbGDSBoundary*>(internal_element));
  } else if (dynamic_cast<_dbGDSPath*>(internal_element) != nullptr) {
    writePath(static_cast<dbGDSPath*>(internal_element));
  } else if (dynamic_cast<_dbGDSSRef*>(internal_element) != nullptr) {
    writeSRef(static_cast<dbGDSSRef*>(internal_element));
  } else if (dynamic_cast<_dbGDSText*>(internal_element) != nullptr) {
    write
