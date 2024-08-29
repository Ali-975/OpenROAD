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

#pragma once

#include <endian.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include "odb/db.h"
#include "odb/gdsUtil.h"

namespace odb {

class dbGDSLib;
class dbGDSElement;
class dbGDSBoundary;
class dbGDSPath;
class dbGDSSRef;
class dbGDSStructure;

class GDSReader
{
 public:
  /**
   * Constructor for GDSReader
   * No operations are performed in the constructor
   */
  GDSReader();

  /**
   * Destructor
   *
   * Does not free the dbGDSLib objects, as they are owned by the database
   */
  ~GDSReader();

  /**
   * Reads a GDS file and returns a dbGDSLib object
   *
   * @param filename The path to the GDS file
   * @param db The database to store the GDS data
   * @return A dbGDSLib object containing the GDS data
   * @throws std::runtime_error if the file cannot be opened, or if the GDS is
   * corrupted
   */
  dbGDSLib* read_gds(const std::string& filename, dbDatabase* db);

 private:
  /** Current filestream */
  std::ifstream _file;
  /** Most recently read record */
  record_t _r;
  /** Current ODB Database */
  dbDatabase* _db;
  /** Current GDS Lib object */
  dbGDSLib* _lib;

  /**
   * Checks if the _r is the expected type
   *
   * Currently functions as an assert statement for the record type.
   * Used after readRecord() to check if the record type is the expected type.
   *
   * @param expect The expected record type
   * @return true if the record type is the expected type
   * @throws std::runtime_error if the record type is not the expected type
   */
  bool checkRType(RecordType expect);

  /**
   * Checks if the _r is the expected data type
   *
   * Currently functions as an assert statement for the data type.
   * Used after readRecord() to check if the data type is the expected type.
   * This function also checks if the data size is the expected size.
   *
   * @param eType The expected data type
   * @param eSize The expected data size
   * @return true if the data type is the expected type
   * @throws std::runtime_error if the data type is not the expected type
   */
  bool checkRData(DataType eType, size_t eSize);

  /**
   * Reads a real8 from _file
   *
   * NOTE: real8 is not the same as double. This conversion is not lossless.
   * @return The real8 read from _file, converted to a double
   */
  double readReal8();

  /** Reads an int32 from _file */
  int32_t readInt32();

  /** Reads an int16 from _file */
  int16_t readInt16();

  /** Reads an int8 from _file */
  int8_t readInt8();

  /**
   * Reads a record from _file and stores it in _r
   *
   * Reads the record type, data type, and length from _file.
   * The data is then read into the appropriate data vector.
   *
   * @return true if a record was read, false if the end of the file was reached
   * @throws std::runtime_error if the record is corrupted
   */
  bool readRecord();

  /** Parses a GDS Lib from the GDS file */
  bool processLib();

  /** Parses a GDS Structure from the GDS file */
  bool processStruct();

  /**
   * Parses a GDS Element from the GDS file
   *
   * @param str The GDS Structure to add the GDS Element to
   */
  bool processElement(dbGDSStructure& str);

  // Specific element types, same as processElement
  dbGDSElement* processBoundary();
  dbGDSElement* processPath();
  dbGDSElement* processSRef();
  dbGDSElement* processText();
  dbGDSElement* processBox();
  dbGDSElement* processNode();

  /**
   * Parses special attributes of a GDS Element
   *
   * @param elem The GDS Element to add the attributes to
   */
  void processPropAttr(dbGDSElement* elem);

  /**
   * Parses the XY data of a GDS Element
   *
   * @param elem The GDS Element to add the XY data to
   * @return true if the XY data was successfully read
   */
  bool processXY(dbGDSElement* elem);

  /**
   * Parses a GDS STrans from the GDS file
   * @return The parsed STrans
   */
  dbGDSSTrans processSTrans();

  /**
   * Parses a GDS Text Presentation from the GDS file
   * @return The parsed STrans
   */
  dbGDSTextPres processTextPres();
};

}  // namespace odb