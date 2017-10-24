#include "morton.h"
int morton(int x, int y, int z, int level)
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

int *code(int morton, int level)//输入一个莫顿序 输出其对应X Y Z编号
{
	int *code = new int[3]{ 0, 0, 0 };
	for (int i = level - 1; i >= 0; i--)
	{
		code[0] += (morton >> (i * 3 + 2)) << i;
		morton -= (morton >> (i * 3 + 2)) << (i * 3 + 2);
		code[1] += (morton >> (i * 3 + 1)) << i;
		morton -= (morton >> (i * 3 + 1)) << (i * 3 + 1);
		code[2] += (morton >> (i * 3)) << i;
		morton -= (morton >> (i * 3)) << (i * 3);
	}
	return code;
}

int *six_n_morton(int morton_code, int level)//计算六领域的莫顿序
{
	int  *six = new int[26];  //0,1，2,3,4,5分别为X+ X- Y+ Y- Z+ Z-
	int *code_ = new int[3];
	code_ = code(morton_code, level);
	six[0] = morton(code_[0] + 1, code_[1], code_[2], level);
	six[1] = morton(code_[0] - 1, code_[1], code_[2], level);
	six[2] = morton(code_[0], code_[1] + 1, code_[2], level);
	six[3] = morton(code_[0], code_[1] - 1, code_[2], level);
	six[4] = morton(code_[0], code_[1], code_[2] + 1, level);
	six[5] = morton(code_[0], code_[1], code_[2] - 1, level);
	six[6] = morton(code_[0] + 1, code_[1] + 1, code_[2] + 1, level);
	six[7] = morton(code_[0] - 1, code_[1] - 1, code_[2] + 1, level);
	six[8] = morton(code_[0] + 1, code_[1] - 1, code_[2] + 1, level);
	six[9] = morton(code_[0] - 1, code_[1] + 1, code_[2] + 1, level);
	six[10] = morton(code_[0] + 1, code_[1], code_[2] + 1, level);
	six[11] = morton(code_[0] - 1, code_[1], code_[2] + 1, level);
	six[12] = morton(code_[0], code_[1] + 1, code_[2] + 1, level);
	six[13] = morton(code_[0], code_[1] - 1, code_[2] + 1, level);
	six[14] = morton(code_[0] + 1, code_[1] + 1, code_[2], level);
	six[15] = morton(code_[0] + 1, code_[1] - 1, code_[2], level);
	six[16] = morton(code_[0] - 1, code_[1] + 1, code_[2], level);
	six[17] = morton(code_[0] - 1, code_[1] - 1, code_[2], level);
	six[18] = morton(code_[0] + 1, code_[1] + 1, code_[2] - 1, level);
	six[19] = morton(code_[0] + 1, code_[1] - 1, code_[2] - 1, level);
	six[20] = morton(code_[0], code_[1] + 1, code_[2] - 1, level);
	six[21] = morton(code_[0], code_[1] - 1, code_[2] - 1, level);
	six[22] = morton(code_[0] - 1, code_[1] + 1, code_[2] - 1, level);
	six[23] = morton(code_[0] - 1, code_[1] - 1, code_[2] - 1, level);
	six[24] = morton(code_[0], code_[1] + 1, code_[2] - 1, level);
	six[25] = morton(code_[0], code_[1] - 1, code_[2] - 1, level);
	delete[] code_;
	return six;
}