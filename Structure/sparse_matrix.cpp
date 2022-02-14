/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */
#include "sparse_matrix.h"

typename SparseMat::Column const& SparseMat::col(std::size_t cid) {
  return column_wise[cid];
}

SparseMat::SparseMat() {
  column_wise.resize(0);
  nnz = 0;
  row_len = 0;
}

SparseMat::SparseMat(std::size_t cols, std::size_t rows) {
  column_wise.resize(cols);
  row_len = rows;
  nnz = 0;
}

bool SparseMat::add_value(std::size_t col, std::size_t row, float value) {
  if (col >= column_wise.size()) column_wise.resize(col + 1);
  if (row >= row_len) row_len = row + 1;
  nnz++;
  Column& column = column_wise[col];
  int before = column.size();
  column.emplace_hint(column.end(), row, value);
  if (before == column.size()) return false;
  return true;
}

void SparseMat::set_value(std::size_t col, std::size_t row, float value) {
  if (col >= column_wise.size()) column_wise.resize(col + 1);
  if (row >= row_len) row_len = row + 1;
  Column& column = column_wise[col];
  column[row] = value;
}

void SparseMat::remove_observation(std::size_t col, std::size_t row) {
  if (col >= column_wise.size()) return;
  Column& column = column_wise[col];
  if (column.find(row) != column.end()) column.erase(row);
  // std::cout << "removing observation: " << col << " " << row << std::endl;
}

void SparseMat::clear() {
  column_wise.clear();
  nnz = 0;
  row_len = 0;
}
