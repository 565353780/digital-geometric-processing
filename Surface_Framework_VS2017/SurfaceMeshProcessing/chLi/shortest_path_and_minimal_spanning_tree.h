#include <iostream>
#include <vector>
#include "MeshDefinition.h"

struct Search_Tree
{
	int vh_idx = -1;
	int heh_idx_to_parent = -1;
	double dist_to_root = 0;

	Search_Tree* parent = NULL;
	Search_Tree* first_child = NULL;
	Search_Tree* left_neighboor = NULL;
	Search_Tree* right_neighboor = NULL;
};

struct Shortest_Path
{
	std::vector<int> vh_idx_set;
	std::vector<int> heh_idx_set;
	double dist = -1;
};

enum Add_State
{
	Success = 0,
	Exist_Better = 1
};

enum Error_Message
{
	Same_Point = -1,
	VH_1_Out_Of_Range = -2,
	VH_2_Out_Of_Range = -3,
	Both_Out_Of_Range = -4,
	Search_Num_Is_Zero = -5,
	Have_Wrong_VH_Idx = -6
};

class Shortest_Path_And_Minimal_Spanning_Tree
{
public:
	Shortest_Path_And_Minimal_Spanning_Tree();
	~Shortest_Path_And_Minimal_Spanning_Tree();

	double get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2);
	double get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2);

	bool is_In_Set(std::vector<int> set, int data);

	Search_Tree* get_Node_by_vh_idx(Search_Tree* tree_node, int vh_idx);

	int get_Search_Tree_Size(Search_Tree* tree_root);

	bool Update_Dist(Search_Tree* tree_node, double update_dist);

	Add_State Add_Child_On(Search_Tree* tree_root, Search_Tree* p_solve, int v_idx, int e_idx_to_parent, double dist);

	bool output_Search_Tree_Node_Message(Search_Tree* tree_node);

	bool test_Search_Tree(Search_Tree* tree_root);

	Shortest_Path get_Total_Tree_HEH_Idx_Set(Search_Tree* tree_root);

	double get_Dist_On_Mesh(Search_Tree* tree_root, int vh_idx);

	Shortest_Path get_Shortest_Path_of_Two_Vertex(Mesh& mesh, int vh_idx_1, int vh_idx_2, int search_num);

	std::vector<Shortest_Path> get_Shortest_Path_Set_Between_All_VH(Mesh& mesh, std::vector<int> vh_idx_set);

	std::vector<Shortest_Path> get_Shortest_Path_Set(Mesh& mesh, std::vector<int> vh_idx_set, int search_num);
};