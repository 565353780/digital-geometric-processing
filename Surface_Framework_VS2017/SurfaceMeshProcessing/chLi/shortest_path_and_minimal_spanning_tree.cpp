#include "shortest_path_and_minimal_spanning_tree.h"

Shortest_Path_And_Minimal_Spanning_Tree::Shortest_Path_And_Minimal_Spanning_Tree()
{

}

Shortest_Path_And_Minimal_Spanning_Tree::~Shortest_Path_And_Minimal_Spanning_Tree()
{

}

double Shortest_Path_And_Minimal_Spanning_Tree::get_Dist_2_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	double dist_2_of_points = 0;

	dist_2_of_points += (point_1.data()[0] - point_2.data()[0]) * (point_1.data()[0] - point_2.data()[0]);
	dist_2_of_points += (point_1.data()[1] - point_2.data()[1]) * (point_1.data()[1] - point_2.data()[1]);
	dist_2_of_points += (point_1.data()[2] - point_2.data()[2]) * (point_1.data()[2] - point_2.data()[2]);

	return dist_2_of_points;
}

double Shortest_Path_And_Minimal_Spanning_Tree::get_Dist_of_Points(Mesh::Point point_1, Mesh::Point point_2)
{
	return sqrt(get_Dist_2_of_Points(point_1, point_2));
}

bool Shortest_Path_And_Minimal_Spanning_Tree::is_In_Set(std::vector<int> set, int data)
{
	for (int i = 0; i < set.size(); ++i)
	{
		if (set[i] == data)
		{
			return true;
		}
	}

	return false;
}

Search_Tree* Shortest_Path_And_Minimal_Spanning_Tree::get_Node_by_vh_idx(Search_Tree* tree_node, int vh_idx)
{
	if (tree_node->vh_idx == vh_idx)
	{
		return tree_node;
	}
	else
	{
		Search_Tree* p_solver = tree_node;

		Search_Tree* found_node;

		if (p_solver->first_child == NULL)
		{
			return NULL;
		}
		else
		{
			p_solver = p_solver->first_child;

			while (p_solver != NULL)
			{
				found_node = get_Node_by_vh_idx(p_solver, vh_idx);

				if (found_node != NULL)
				{
					return found_node;
				}

				p_solver = p_solver->right_neighboor;
			}
		}
	}

	return NULL;
}

int Shortest_Path_And_Minimal_Spanning_Tree::get_Search_Tree_Size(Search_Tree* tree_root)
{
	if (tree_root->first_child == NULL)
	{
		return 1;
	}
	else
	{
		Search_Tree* p_solver = tree_root;

		int tree_size = 0;

		p_solver = p_solver->first_child;

		while (p_solver != NULL)
		{
			tree_size += get_Search_Tree_Size(p_solver);

			p_solver = p_solver->right_neighboor;
		}

		return tree_size + 1;
	}
}

bool Shortest_Path_And_Minimal_Spanning_Tree::Update_Dist(Search_Tree* tree_node, double update_dist)
{
	Search_Tree* p_solver = tree_node;

	tree_node->dist_to_root += update_dist;

	if (tree_node->dist_to_root < 0)
	{
		std::cout << "fudefude~~~~~~~~" << std::endl;
	}

	if (tree_node->first_child == NULL)
	{
		return true;
	}
	else
	{
		p_solver = p_solver->first_child;

		while (p_solver != NULL)
		{
			Update_Dist(p_solver, update_dist);

			p_solver = p_solver->right_neighboor;
		}
	}

	return true;
}

Add_State Shortest_Path_And_Minimal_Spanning_Tree::Add_Child_On(Search_Tree* tree_root, Search_Tree* tree_node, int vh_idx, int heh_idx_to_parent, double dist)
{
	Search_Tree* search_node = get_Node_by_vh_idx(tree_root, vh_idx);

	if (search_node == NULL)
	{
		if (tree_node->first_child == NULL)
		{
			tree_node->first_child = new Search_Tree();

			tree_node->first_child->vh_idx = vh_idx;
			tree_node->first_child->heh_idx_to_parent = heh_idx_to_parent;
			tree_node->first_child->dist_to_root = tree_node->dist_to_root + dist;

			tree_node->first_child->parent = tree_node;

			return Success;
		}
		else
		{
			Search_Tree* p_solver = tree_node->first_child;

			while (p_solver->right_neighboor != NULL)
			{
				p_solver = p_solver->right_neighboor;
			}

			p_solver->right_neighboor = new Search_Tree();

			p_solver->right_neighboor->vh_idx = vh_idx;
			p_solver->right_neighboor->heh_idx_to_parent = heh_idx_to_parent;
			p_solver->right_neighboor->dist_to_root = tree_node->dist_to_root + dist;

			p_solver->right_neighboor->parent = tree_node;
			p_solver->right_neighboor->left_neighboor = p_solver;

			p_solver = NULL;

			return Success;
		}
	}
	else if (search_node->dist_to_root < tree_node->dist_to_root + dist)
	{
		return Exist_Better;
	}
	else
	{
		if (search_node->left_neighboor == NULL)
		{
			if (search_node->right_neighboor == NULL)
			{
				search_node->parent->first_child = NULL;
			}
			else
			{
				search_node->parent->first_child = search_node->right_neighboor;
				search_node->right_neighboor->left_neighboor = NULL;
			}
		}
		else
		{
			if (search_node->right_neighboor == NULL)
			{
				search_node->left_neighboor->right_neighboor = NULL;
			}
			else
			{
				search_node->left_neighboor->right_neighboor = search_node->right_neighboor;
				search_node->right_neighboor->left_neighboor = search_node->left_neighboor;
			}
		}

		search_node->right_neighboor = NULL;

		double update_dist = tree_node->dist_to_root + dist - search_node->dist_to_root;

		if (tree_node->first_child == NULL)
		{
			tree_node->first_child = search_node;

			search_node->vh_idx = vh_idx;
			search_node->heh_idx_to_parent = heh_idx_to_parent;

			search_node->parent = tree_node;
			search_node->left_neighboor = NULL;
		}
		else
		{
			Search_Tree* p_solver = tree_node->first_child;

			while (p_solver->right_neighboor != NULL)
			{
				p_solver = p_solver->right_neighboor;
			}

			p_solver->right_neighboor = search_node;

			search_node->vh_idx = vh_idx;
			search_node->heh_idx_to_parent = heh_idx_to_parent;

			search_node->parent = tree_node;
			search_node->left_neighboor = p_solver;
		}

		Update_Dist(search_node, update_dist);

		return Success;
	}
}

bool Shortest_Path_And_Minimal_Spanning_Tree::output_Search_Tree_Node_Message(Search_Tree* tree_node)
{
	std::cout << "=====================================" << std::endl;

	if (tree_node->parent == NULL)
	{
		std::cout << "\t\tNULL" << std::endl;
	}
	else
	{
		std::cout << "\t\t" << tree_node->parent->vh_idx << std::endl;
	}

	std::cout << std::endl;

	std::cout << "-------------------------------------" << std::endl;

	std::cout << "\t|" << "vh:" << tree_node->vh_idx << "\t\t|" << std::endl;

	if (tree_node->left_neighboor == NULL)
	{
		std::cout << "NULL";
	}
	else
	{
		std::cout << tree_node->left_neighboor->vh_idx;
	}

	std::cout << "\t|heh:" << tree_node->heh_idx_to_parent << "\t\t|\t";

	if (tree_node->right_neighboor == NULL)
	{
		std::cout << "NULL" << std::endl;
	}
	else
	{
		std::cout << tree_node->right_neighboor->vh_idx << std::endl;
	}

	std::cout << "\t|dist:" << tree_node->dist_to_root;

	if (tree_node->dist_to_root == 0)
	{
		std::cout << "\t\t|" << std::endl;
	}
	else
	{
		std::cout << "\t|" << std::endl;
	}

	std::cout << "-------------------------------------" << std::endl;

	std::cout << std::endl;

	if (tree_node->first_child == NULL)
	{
		std::cout << "\t\tNULL" << std::endl;
	}
	else
	{
		std::cout << "\t\t" << tree_node->first_child->vh_idx << std::endl;
	}

	return true;
}

bool Shortest_Path_And_Minimal_Spanning_Tree::test_Search_Tree(Search_Tree* tree_root)
{
	Search_Tree* search_node = tree_root;

	std::string test_btn = "e";
	std::cin >> test_btn;

	while (test_btn != "q")
	{
		if (test_btn == "w")
		{
			if (search_node->parent != NULL)
			{
				search_node = search_node->parent;
			}
			
		}
		else if (test_btn == "a")
		{
			if (search_node->left_neighboor != NULL)
			{
				search_node = search_node->left_neighboor;
			}
		}
		else if (test_btn == "s")
		{
			if (search_node->first_child != NULL)
			{
				search_node = search_node->first_child;
			}
		}
		else if (test_btn == "d")
		{
			if (search_node->right_neighboor != NULL)
			{
				search_node = search_node->right_neighboor;
			}
		}

		output_Search_Tree_Node_Message(search_node);

		std::cin >> test_btn;
	}

	return true;
}

Shortest_Path Shortest_Path_And_Minimal_Spanning_Tree::get_Total_Tree_HEH_Idx_Set(Search_Tree* tree_root)
{
	Shortest_Path total_shortest_path_set;

	total_shortest_path_set.vh_idx_set.emplace_back(tree_root->vh_idx);

	if (tree_root->parent != NULL)
	{
		total_shortest_path_set.heh_idx_set.emplace_back(tree_root->heh_idx_to_parent);
	}

	if (tree_root->first_child == NULL)
	{
		return total_shortest_path_set;
	}
	else
	{
		Search_Tree* p_solver = tree_root;

		p_solver = p_solver->first_child;

		while (p_solver != NULL)
		{
			Shortest_Path current_shortest_path_set = get_Total_Tree_HEH_Idx_Set(p_solver);

			for (int i = 0; i < current_shortest_path_set.heh_idx_set.size(); ++i)
			{
				total_shortest_path_set.heh_idx_set.emplace_back(current_shortest_path_set.heh_idx_set[i]);
			}

			p_solver = p_solver->right_neighboor;
		}
	}

	return total_shortest_path_set;
}

double Shortest_Path_And_Minimal_Spanning_Tree::get_Dist_On_Mesh(Search_Tree* tree_root, int vh_idx)
{
	Search_Tree* search_node = get_Node_by_vh_idx(tree_root, vh_idx);

	if (search_node != NULL)
	{
		return search_node->dist_to_root;
	}
	else
	{
		return -1;
	}
}

Shortest_Path Shortest_Path_And_Minimal_Spanning_Tree::get_Shortest_Path_of_Two_Vertex(Mesh& mesh, int vh_idx_1, int vh_idx_2, int search_num)
{
	if (search_num == 0)
	{
		Shortest_Path error_msg;

		error_msg.heh_idx_set.emplace_back(Search_Num_Is_Zero);

		return error_msg;
	}

	if (vh_idx_1 == vh_idx_2)
	{
		Shortest_Path error_msg;

		error_msg.heh_idx_set.emplace_back(Same_Point);

		return error_msg;
	}
	else if (vh_idx_1 < 0 || vh_idx_1 >= mesh.n_vertices())
	{
		if (vh_idx_2 < 0 || vh_idx_2 >= mesh.n_vertices())
		{
			Shortest_Path error_msg;

			error_msg.heh_idx_set.emplace_back(Both_Out_Of_Range);

			return error_msg;
		}
		else
		{
			Shortest_Path error_msg;

			error_msg.heh_idx_set.emplace_back(VH_1_Out_Of_Range);

			return error_msg;
		}
	}
	else if (vh_idx_2 < 0 || vh_idx_2 >= mesh.n_vertices())
	{
		Shortest_Path error_msg;

		error_msg.heh_idx_set.emplace_back(VH_2_Out_Of_Range);

		return error_msg;
	}

	Search_Tree* search_tree_from_vh_1 = new Search_Tree();

	Search_Tree* p_solver = search_tree_from_vh_1;

	p_solver->vh_idx = vh_idx_1;

	std::vector<int> current_search_vh_idx_set;
	std::vector<int> new_current_search_vh_idx_set;

	current_search_vh_idx_set.emplace_back(vh_idx_1);

	int test_num = 0;

	if (search_num != 0)
	{
		//while (!is_In_Set(current_search_vh_idx_set, vh_idx_2) && test_num < search_num)
		while (get_Search_Tree_Size(search_tree_from_vh_1) < mesh.n_vertices() && (test_num < search_num || search_num == -1))
		{
			++test_num;

			new_current_search_vh_idx_set.clear();

			for (int i = 0; i < current_search_vh_idx_set.size(); ++i)
			{
				p_solver = get_Node_by_vh_idx(search_tree_from_vh_1, current_search_vh_idx_set[i]);

				Mesh::VertexHandle current_vh = mesh.vertex_handle(current_search_vh_idx_set[i]);

				Mesh::Point current_vh_point = mesh.point(current_vh);

				Mesh::VertexVertexIter vvit;

				Mesh::Point search_vh_point;

				bool finished = false;

				for (vvit = mesh.vv_iter(current_vh); vvit->is_valid(); ++vvit)
				{
					if (vvit->idx() == mesh.vv_iter(current_vh)->idx())
					{
						if (!finished)
						{
							finished = true;
						}
						else
						{
							break;
						}
					}

					search_vh_point = mesh.point(*vvit);

					Mesh::HalfedgeHandle current_heh = mesh.find_halfedge(current_vh, *vvit);

					double dist = get_Dist_of_Points(current_vh_point, search_vh_point);

					Add_State add_state = Add_Child_On(search_tree_from_vh_1, p_solver, vvit->idx(), current_heh.idx(), dist);

					if (add_state == Success && !is_In_Set(new_current_search_vh_idx_set, vvit->idx()))
					{
						new_current_search_vh_idx_set.emplace_back(vvit->idx());
					}
				}
			}

			current_search_vh_idx_set.resize(new_current_search_vh_idx_set.size());

			for (int i = 0; i < current_search_vh_idx_set.size(); ++i)
			{
				current_search_vh_idx_set[i] = new_current_search_vh_idx_set[i];
			}
		}
	}

	//test_Search_Tree(search_tree_from_vh_1);

	if (get_Search_Tree_Size(search_tree_from_vh_1) == mesh.n_vertices())
	{
		Shortest_Path shortest_path;

		Search_Tree* search_node = get_Node_by_vh_idx(search_tree_from_vh_1, vh_idx_2);

		shortest_path.dist = search_node->dist_to_root;

		while (search_node->parent != NULL)
		{
			shortest_path.vh_idx_set.emplace_back(search_node->vh_idx);
			shortest_path.heh_idx_set.emplace_back(search_node->heh_idx_to_parent);

			search_node = search_node->parent;
		}

		shortest_path.vh_idx_set.emplace_back(search_node->vh_idx);

		return shortest_path;
	}
	else
	{
		Shortest_Path total_shortest_path_set = get_Total_Tree_HEH_Idx_Set(search_tree_from_vh_1);

		return total_shortest_path_set;
	}
}

std::vector<Shortest_Path> Shortest_Path_And_Minimal_Spanning_Tree::get_Shortest_Path_Set_Between_All_VH(Mesh& mesh, std::vector<int> vh_idx_set)
{
	std::vector<Shortest_Path> shortest_path_set;

	for (int i = 0; i < vh_idx_set.size(); ++i)
	{
		if (vh_idx_set[i] < 0 || vh_idx_set[i] >= mesh.n_vertices())
		{
			Shortest_Path error_msg;

			error_msg.heh_idx_set.emplace_back(Have_Wrong_VH_Idx);

			shortest_path_set.emplace_back(error_msg);

			return shortest_path_set;
		}
	}

	for (int i = 0; i < vh_idx_set.size(); ++i)
	{
		for (int j = 0; j < i; ++j)
		{
			shortest_path_set.emplace_back(get_Shortest_Path_of_Two_Vertex(mesh, vh_idx_set[i], vh_idx_set[j], -1));
		}
	}

	std::vector<int> mst_vh_idx_set;

	std::vector<int> mst_path_idx_set;

	while (mst_vh_idx_set.size() < vh_idx_set.size())
	{
		int min_dist_idx = -1;
		double min_dist = -1;

		for (int i = 0; i < shortest_path_set.size(); ++i)
		{
			if (mst_path_idx_set.size() == 0)
			{
				if (shortest_path_set[i].dist < min_dist || min_dist == -1)
				{
					min_dist_idx = i;
					min_dist = shortest_path_set[i].dist;
				}
			}
			else if (is_In_Set(mst_vh_idx_set, shortest_path_set[i].vh_idx_set[0]))
			{
				if (!is_In_Set(mst_vh_idx_set, shortest_path_set[i].vh_idx_set[shortest_path_set[i].vh_idx_set.size() - 1]))
				{
					if (shortest_path_set[i].dist < min_dist || min_dist == -1)
					{
						min_dist_idx = i;
						min_dist = shortest_path_set[i].dist;
					}
				}
			}
			else if (is_In_Set(mst_vh_idx_set, shortest_path_set[i].vh_idx_set[shortest_path_set[i].vh_idx_set.size() - 1]))
			{
				if (shortest_path_set[i].dist < min_dist || min_dist == -1)
				{
					min_dist_idx = i;
					min_dist = shortest_path_set[i].dist;
				}
			}
		}

		if (min_dist_idx != -1)
		{
			mst_path_idx_set.emplace_back(min_dist_idx);

			int vh_idx_1 = shortest_path_set[min_dist_idx].vh_idx_set[0];
			int vh_idx_2 = shortest_path_set[min_dist_idx].vh_idx_set[shortest_path_set[min_dist_idx].vh_idx_set.size() - 1];

			if (!is_In_Set(mst_vh_idx_set, vh_idx_1))
			{
				mst_vh_idx_set.emplace_back(vh_idx_1);
			}
			
			if (!is_In_Set(mst_vh_idx_set, vh_idx_2))
			{
				mst_vh_idx_set.emplace_back(vh_idx_2);
			}
		}
	}

	std::vector<Shortest_Path> mst_path_set;

	for (int i = 0; i < mst_path_idx_set.size(); ++i)
	{
		mst_path_set.emplace_back(shortest_path_set[mst_path_idx_set[i]]);
	}

	return mst_path_set;
}

std::vector<Shortest_Path> Shortest_Path_And_Minimal_Spanning_Tree::get_Shortest_Path_Set(Mesh& mesh, std::vector<int> vh_idx_set, int search_num)
{
	if (vh_idx_set.size() == 2)
	{
		std::vector<Shortest_Path> shortest_path_set;

		shortest_path_set.emplace_back(get_Shortest_Path_of_Two_Vertex(mesh, vh_idx_set[0], vh_idx_set[1], search_num));

		return shortest_path_set;
	}
	else if (vh_idx_set.size() > 2)
	{
		return get_Shortest_Path_Set_Between_All_VH(mesh, vh_idx_set);
	}
}