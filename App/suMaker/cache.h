#pragma once


#include <string>
#include <deque>
#include <sstream>
#include <fstream>
using namespace std;
#ifndef voxels_cache
#define voxels_cache
template <class T>//Ҫ����==  =(liangge)  !=  < >�����     ������һ����       T=-1Ҫ���سɱ�Ǵε�Ԫ������-
class voxels {//�������ֻ�ṩ�������� getCode��saveOne   ����������Ϊprivate
private:
	string address;
	int level;
	int levelL, levelH;//levelL��ÿһ��ķָ����    levelH���ܵķָ����
public://������sizeҪС�ڵ���levelL������һ���ļ��е���Ŀ    �����м���һ�����  ������� �͸�С   û�м���� ��Ҳ��֪��Ϊʲô  �����ǲ���ʱ��ֻ��һ���ļ�  �ڶ����ļ��򲻿��ͳ�����  ���ļ�����ʱ��Ҫע��
	deque<T> voxel;
	voxels(string address_, int level, int size = 4096);//���캯����Ҫ֪��ȥ������ļ�(Ŀ¼)����ֵ��address  ���Ҷ���ǰsize��ֵ    ������address��deque�Ĵ�С,�����������ִ���   
	voxels(string address_, int level, string sth, int size = 4096);//ȫ���ܹ��캯��
	typename deque<T>::iterator getCode(unsigned long long mortonCode);//����deque����������Ĵ���  ����з����������  ���û�о�ȥ��һ��
	typename deque<T>::iterator readOneCode(T needed);//����Ϊ��Ҫ��   �������һ�������� ���û���ҵ��ͷ���end()
	typename deque<T>::iterator readOneToUpdate(unsigned long long lordCommander);//����Ϊ��Ҫ����  ��һ��ɾ��һ��  ɾ��ʱ��Ҫ���ļ��еĽ����޸�  ���ض�������ݵĵ�����
	T readOneFromFile(unsigned long long monica);//����monica��ŵ�����
	void saveOne(T oneToSave);//���������������  ������Ӧ�������  û�еĻ�˵�����ᱬը
	void updateOne(T & lokTar);
	~voxels() {
		for (int i = 0;i < voxel.size();i++) {
			updateOne(voxel[i]);
		}
	}
};

template<class T>
inline void voxels<T>::updateOne(T & lokTar)
{
	string fileAddress;
	stringstream ss;
	ss << level;
	ss >> fileAddress;
	fileAddress = address + "\\outOfCore" + fileAddress + ".bat";
	fstream write;
	write.open(fileAddress, ios::out | ios::in | ios::binary);
	unsigned long long location;
	location = lokTar % (1 << 3 * level);
	//cout << "!!!!!" << location << ' ' << lokTar;
	write.seekp(sizeof(T)*location, ios::beg);
	write.write((char*)&lokTar, sizeof(T));
	write.close();


	ifstream readIn;
	readIn.open(fileAddress, ios::in | ios::binary);
	T majnun;
	/*for (int i = 0;i < 64;i++) {
	readIn.seekg(sizeof(T)*i, ios::beg);

	readIn.read((char *)&majnun, sizeof(T));
	cout << ' ' << majnun << endl;
	}*/
	readIn.close();

}


template<class T>
inline void voxels<T>::saveOne(T oneToSave) {
	typename deque<T>::iterator temp = readOneCode(oneToSave);
	if (temp != voxel.end()) {
		*temp = oneToSave;
	}
	else
		*readOneToUpdate(oneToSave.morton) = oneToSave;

}
template<class T>
inline T voxels<T>::readOneFromFile(unsigned long long monica) {
	string fileAddress;
	stringstream ss;
	ss << level;
	ss >> fileAddress;
	fileAddress = address + "\\outOfCore" + fileAddress + ".bat";
	ifstream readIn;
	readIn.open(fileAddress, ios::in | ios::binary);
	T majnun;
	readIn.seekg(sizeof(T)*monica, ios::beg);

	readIn.read((char *)&majnun, sizeof(T));

	readIn.close();
	return majnun;
}
template<class T>
inline typename deque<T>::iterator voxels<T>::readOneToUpdate(unsigned long long lordCommander) {
	T majnun = readOneFromFile(lordCommander);
	updateOne(voxel[0]);//upadte the file
	voxel.pop_front();
	voxel.push_back(majnun);
	return (voxel.end() - 1);
}
template<class T>
inline typename deque<T>::iterator voxels<T>::getCode(unsigned long long mortonCode) {
	unsigned long long findThisOne = mortonCode;
	T temp_;
	temp_.morton = mortonCode;
	deque<T>::iterator temp = readOneCode(temp_);
	if (temp != voxel.end())
		return temp;
	else
		return readOneToUpdate(findThisOne);
}
template<class T>
inline voxels<T>::voxels(string address_, int level_, int size = 4096) {
	address = address_;
	level = level_;
	size = size < 1 << (3 * level) ? size : 1 << (3 * level);
	for (int i = 0; i < size; i++) {
		voxel.push_back(readOneFromFile(i));
		//cout << voxel[i] << endl;
	}
	cout << voxel.size();
}

template<class T>
inline voxels<T>::voxels(string address_, int level_, string sth, int size = 4096)
{
	level = level_;
	address = address_;
	size = size < 1 << (3 * level) ? size : 1 << (3 * level);


	string fileAddress;
	stringstream ss;
	ss << level;
	ss >> fileAddress;
	fileAddress = address + "\\outOfCore" + fileAddress + ".bat";

	fstream newfiles;
	newfiles.open(fileAddress, ios::out | ios::in | ios::binary);
	VoxelInfo temp;
	for (int i = 0;i < (1 << (level * 3));i++) {
		temp.morton = i;
		newfiles.write((char *)&temp, sizeof(VoxelInfo));
	}
	newfiles.close();





	for (int i = 0; i < size; i++) {
		voxel.push_back(readOneFromFile(i));
		//cout << voxel[i] << endl;
	}
	cout << voxel.size();
}


template<class T>
inline typename deque<T>::iterator voxels<T>::readOneCode(T needed) {
	deque<T>::iterator temp = voxel.begin();
	for (; temp != voxel.end();temp++) {
		if (*temp == needed) {
			return temp;
		}
	}
	return voxel.end();
}
#endif