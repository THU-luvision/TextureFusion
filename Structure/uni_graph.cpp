/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <limits>
#include <list>

#include "uni_graph.h"

UniGraph::UniGraph(std::size_t nodes) {
    adj_lists.resize(nodes);
    labels.resize(nodes);
    edges = 0;
}


bool UniGraph::add_node(chisel::ChunkID id){;
	if ((chunks.find(id))!= chunks.end()) return false;
	chunks.insert(std::make_pair(id, adj_lists.size()));
    adj_lists.emplace_back(std::vector<std::size_t>());
    labels.emplace_back(0);
	return true;
}

void UniGraph::add_edge_by_node(chisel::ChunkID id){
	if (chunks.find(id) == chunks.end()) return;
	auto it = chunks.find(id);
	std::size_t n1 = it -> second;

	for (int i = 0; i < 6; i++)
	{
		if ((it = chunks.find(id + chisel::neighbourhood[i]))!= chunks.end()) {add_edge(n1, it -> second); add_edge(it -> second, n1);}
	}
}

void UniGraph::add_edge_by_node(chisel::ChunkID id, bool flag[6]){
	if (chunks.find(id) == chunks.end()) return;
	auto it = chunks.find(id);
	std::size_t n1 = it -> second;
	
	for (int i = 0; i < 6; i++)
	{
		if (flag[i] && (it = chunks.find(id + chisel::neighbourhood[i]))!= chunks.end()) {add_edge(n1, it -> second); add_edge(it -> second, n1);}
	}
#if 0
	for (int i = 0; i < 6; i++)
	{
		for (int j = i+1; j < 6; j++)
		{
			if (flag[i] && flag[j] && (it = chunks.find(id + chisel::neighbourhood[i]
			 +chisel::neighbourhood[j]))!= chunks.end()) {add_edge(n1, it -> second); add_edge(it -> second, n1);}
		}
	}
	for (int i = 0; i < 6; i++)
	{
		for (int j = i+1; j < 6; j++)
		{
			for (int k = j+1; k < 6; k++)
			{
				if (flag[i] && flag[j] && flag[k] && (it = chunks.find(id + chisel::neighbourhood[i]
			 	+chisel::neighbourhood[j]+ chisel::neighbourhood[k]))!= chunks.end()) {add_edge(n1, it -> second); add_edge(it -> second, n1);}
			}
		}
	}
#endif
}

void UniGraph::get_subgraphs(std::vector<std::size_t> index, std::vector<std::vector<std::size_t>>& subgraphs){
    std::vector<bool> concerned(adj_lists.size(), false);
    for(std::size_t i = 0; i < index.size(); i++){
        concerned[index[i]] = true;   
    }
    for(std::size_t i = 0; i < adj_lists.size(); i++) {
        if (!concerned[i]) continue;
        std::vector<std::size_t> adj_list;
        for(std::size_t j = 0; j < adj_lists[i].size(); j++){
            std::size_t node = adj_lists[i][j];
            if (concerned[node]) adj_list.emplace_back(node);
        }
        subgraphs.emplace_back(adj_list);
    }
}

void UniGraph::remove_node(chisel::ChunkID id)
{
    if (chunks.find(id) == chunks.end()) return;
    std::size_t n = chunks.find(id) -> second;
    std::vector<std::size_t> adj = adj_lists[n];
    for (int i = 0; i < adj.size();i++)
    {
        std::size_t hot = adj[i];
        for (int j = 0; j < adj_lists[hot].size(); j++)
        {
           if (adj_lists[hot][j] == n)
           {
               adj_lists[hot].erase(adj_lists[hot].begin()+j);
               j--;
           }
        }
    }
    adj_lists[n].clear();
}

/*
void UniGraph::get_subgraphs(std::size_t label, std::vector<std::vector<std::size_t> > * subgraphs){
    
    std::vector<bool> used(adj_lists.size(), false);

    for(std::size_t i = 0; i < adj_lists.size(); ++i) {
        if (labels[i] == label && !used[i]) {
            subgraphs->push_back(std::vector<std::size_t>());

            std::list<std::size_t> queue;

            queue.push_back(i);
            used[i] = true;

            while (!queue.empty()) {
                std::size_t node = queue.front();
                queue.pop_front();

                subgraphs->back().push_back(node);

                // Add all unused neighbours with the same label to the queue.
                std::vector<std::size_t> const & adj_list = adj_lists[node];
                for(std::size_t j = 0; j < adj_list.size(); ++j) {
                    std::size_t adj_node = adj_list[j];
                    assert(adj_node < labels.size() && adj_node < used.size());
                    if (labels[adj_node] == label && !used[adj_node]){
                        queue.push_back(adj_node);
                        used[adj_node] = true;
                    }
                }
            }
        }
    }
}
*/