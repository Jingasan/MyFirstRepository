/***** 3次元点群の読み込み *****/


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>		// 点群のデータ型
#include <pcl/io/pcd_io.h>			// PCD データ入出力
#include <pcl/io/ply_io.h>			// PLY データ入出力
#include <pcl/common/io.h>			// 入出力

/* 作成ヘッダファイル */
#include "FunctionDefinition.h"	// 関数の定義


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** PCD の読み込み( PointXYZRGB ) ***/
int loadPCDPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PCD データ( PointXYZRGB )の読込中" << "\r";
	if( pcl::io::loadPCDFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD データの読み込み失敗。" << endl; return 1; }
	cout << ">>> PCD データの読み込み OK!" << endl;

	return 0;
}


/*** PCD 点群の読み込み( PointXYZ ) ***/
int loadPCDPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PCD データ( PointXYZ )の読込中" << "\r";
	if( pcl::io::loadPCDFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD データの読み込み失敗。" << endl; return 1; }
	cout << ">>> PCD データの読み込み OK!" << endl;

	return 0;
}


/*** PLY 点群の読み込み( PointXYZRGB ) ***/
int loadPLYPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PLY データ( PointXYZRGB )の読込中" << "\r";
	if( pcl::io::loadPLYFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD データの読み込み失敗。" << endl; return 1; }
	cout << ">>> PLY データの読み込み OK!" << endl;

	return 0;
}


/*** PLY 点群の読み込み( PointXYZ ) ***/
int loadPLYPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PLY データ( PointXYZ )の読込中" << "\r";
	if( pcl::io::loadPLYFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD データの読み込み失敗。" << endl; return 1; }
	cout << ">>> PLY データの読み込み OK!" << endl;

	return 0;
}

