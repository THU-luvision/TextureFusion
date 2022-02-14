/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

//specially designed for Real-time MRF problem
#ifndef TEXMAP_SparseMat_H
#define TEXMAP_SparseMat_H

#include <vector>
#include <map>
#include <algorithm>
#include <iostream>

typedef std::pair<size_t, float> NodeCost;

class SparseMat 
{
    public:
        typedef std::map<std::size_t, float> Column;
    private:
        std::vector<Column> column_wise;
        std::size_t nnz;
	 	std::size_t row_len;
    public:
        SparseMat();
        SparseMat(std::size_t cols, std::size_t rows);

        std::size_t cols();
        std::size_t rows();
		std::size_t get_nnz();

        Column const & col(std::size_t cid);

        bool add_value(std::size_t col, std::size_t row, float value);
		void set_value(std::size_t col, std::size_t row, float value);
		void resize(std::size_t cols);
		void clear();
        void remove_node(std::size_t cid);
        void remove_observation(std::size_t cid, std::size_t row);
};

inline std::size_t SparseMat::cols()
{
	return column_wise.size();	
}

inline std::size_t SparseMat::rows()
{
	return row_len;	
}

inline std::size_t SparseMat::get_nnz()
{
	return nnz;	
}

inline void SparseMat::resize(std::size_t cols)
{
    column_wise.resize(cols);	
}

inline void SparseMat::remove_node(std::size_t cid){
    if (cid >= column_wise.size()) return;
    column_wise[cid].clear();
}

#endif /*TEXMAP_SparseMat_H */