#pragma once

#ifndef  morton_fun
// ! morton_fun
#define morton_fun
int morton(int x, int y, int z, int level);//����Ī����
int *code(int morton, int level);//����Ī�����ֵ ���XYZ��ֵ
int *six_n_morton(int morton_code, int level);//��������Ī����ֵ

#endif 