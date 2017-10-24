#pragma once

//morton tool class
class suMorton
{
public:
	static int encode(int x, int y, int z, int level)
	{
		int morton = 0;
		for (int i = level - 1; i >= 0; i--)
		{
			morton += (x >> i) << (3 * i + 2);
			x = x - ((x >> i) << i);
			morton += (y >> i) << (3 * i + 1);
			y = y - ((y >> i) << i);
			morton += (z >> i) << (3 * i);
			z = z - ((z >> i) << i);
		}
		return morton;
	}
	static void decode(int &x, int &y, int &z, int morton, int level)
	{		
		for (int i = level - 1; i >= 0; i--)
		{
			x += (morton >> (i * 3 + 2)) << i;
			morton -= (morton >> (i * 3 + 2)) << (i * 3 + 2);
			y += (morton >> (i * 3 + 1)) << i;
			morton -= (morton >> (i * 3 + 1)) << (i * 3 + 1);
			z += (morton >> (i * 3)) << i;
			morton -= (morton >> (i * 3)) << (i * 3);
		}
		return;
	}

	static int get_6neighbors(std::vector<int> &neighbors, int morton_code, int level)  //return 6 neighbors
	{
		int const nn[6][3] = {
			{ 1, 0, 0},
			{ -1, 0, 0},
			{ 0, 1, 0},
			{ 0, -1, 0},
			{ 0, 0, 1},
			{ 0, 0, -1}
		};
		int x, y, z;
		decode(x, y, z, morton_code, level);
		neighbors.clear();
		for (int i = 0; i < 6; i++)
		{
			neighbors.push_back(encode(x + nn[i][0], y + nn[i][1], z + nn[i][2], level) );
		}
		return neighbors.size();
	}
public:

};
