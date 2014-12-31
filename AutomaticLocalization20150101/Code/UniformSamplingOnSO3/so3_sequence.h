/***** 回転空間の均等なサンプリング( ヘッダファイル ) *****/


/*** 定数の定義 ***/
#define M_PI 3.141592	// 円周率


/*** インクルードファイル ***/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
//#include <stdbool.h>


/*** 名前空間の宣言 ***/
using namespace std;


/*** 関数の定義 ***/
void pix2ang_nest( long, long, double*, double* );
void mk_pix2xy( int *, int* );
vector<double> find_point( int, long, long, long, double );
bool hopf2quat( vector< vector<double> > );
