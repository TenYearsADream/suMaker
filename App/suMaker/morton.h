#pragma once

#ifndef  morton_fun
// ! morton_fun
#define morton_fun
int morton(int x, int y, int z, int level);//计算莫顿序
int *code(int morton, int level);//输入莫顿序的值 输出XYZ的值
int *six_n_morton(int morton_code, int level);//输出领域的莫顿序值

#endif 