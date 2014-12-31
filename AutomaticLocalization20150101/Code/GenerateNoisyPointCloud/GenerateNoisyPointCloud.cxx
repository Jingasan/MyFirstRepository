/***** 入力点群に対するランダムノイズの散布 *****/

// 【 コマンドライン引数 】
// 指定するコマンドライン引数
// 1. 入力点群ファイル名1.pcd
// 2. 入力点群ファイル名2.pcd
// 3. 出力点群ファイル名3.pcd
// [ 4. 散布するランダムノイズ点群数 ]

// 例 1：
// Input/PointData/ModelData/LiverR20/0001.pcd Output/NoisyPointData/ModelData/LiverR20/0001N200.pcd Output/NoisyPointData/ModelData/LiverR20/0001N200.ply 200

// 例 2：
// Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.pcd Output/NoisyPointData/ModelData/LiverR20/0001_1GaussianN200.pcd Output/NoisyPointData/ModelData/LiverR20/0001_1GaussianN200.ply 200


/*** インクルードファイル ***/

/* C */
#include <float.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char *argv[] ){
	

	/* コマンドライン引数読み込み失敗時 */
	if( argc < 4 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << " inputPointCloud.pcd outputPointCloud.pcd outputPointCloud.ply [ noiseNum ]" << endl;
		return EXIT_FAILURE;
	}


	/*** 変数の宣言 ***/
	int noiseNum = NOISE_NUM;
	const char* inputFileName = argv[ 1 ];		// 入力点群ファイル名
	const char* outputFileName1 = argv[ 2 ];	// 出力点群ファイル名
	const char* outputFileName2 = argv[ 3 ];	// 出力点群ファイル名
	if( argc > 4 ) noiseNum = atoi( argv[ 4 ] );
	double maxX = - DBL_MAX;	// 入力点群座標の X 最大値
	double maxY = - DBL_MAX;	// 入力点群座標の Y 最大値
	double maxZ = - DBL_MAX;	// 入力点群座標の Z 最大値
	double minX = DBL_MAX;		// 入力点群座標の X 最小値
	double minY = DBL_MAX;		// 入力点群座標の Y 最小値
	double minZ = DBL_MAX;		// 入力点群座標の Z 最小値
	double widthX;					// 入力点群の X 方向幅
	double widthY;					// 入力点群の Y 方向幅
	double widthZ;					// 入力点群の Z 方向幅


	/*** ランダムノイズ散布対象点群の読み込み ***/

	/* 入力点群のメモリ確保 */
	cout << ">>> 入力点群( PointXYZRGB )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1( new pcl::PointCloud< pcl::PointXYZRGB > );
	if( pcl::io::loadPCDFile( inputFileName, *cloud1 ) == -1 ){ cout << ">>> 入力点群データの読み込み失敗" << endl; return 1; }
	cout << ">>> 入力点群( PointXYZRGB )の読み込み OK" << endl;

	/* 入力点群の端点の取得 */
	for( int i = 0; i < cloud1->points.size(); i++ ){

		/* X, Y, Z の最小値を設定 */
		if( cloud1->points[ i ].x < minX ) minX = cloud1->points[ i ].x;
		if( cloud1->points[ i ].y < minY ) minY = cloud1->points[ i ].y;
		if( cloud1->points[ i ].z < minZ ) minZ = cloud1->points[ i ].z;
		
		/* X, Y, Z の最大値を設定 */
		if( cloud1->points[ i ].x > maxX ) maxX = cloud1->points[ i ].x;
		if( cloud1->points[ i ].y > maxY ) maxY = cloud1->points[ i ].y;
		if( cloud1->points[ i ].z > maxZ ) maxZ = cloud1->points[ i ].z;
	}

	/* 入力点群幅の取得 */
	widthX = maxX - minX;	// 点群の X 方向幅
	widthY = maxY - minY;	// 点群の Y 方向幅
	widthZ = maxZ - minZ;	// 点群の Z 方向幅


	/*** ノイズ点群の作成 ***/

	/* ノイズ点群のメモリ確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr noise1( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointXYZRGB temp;

	/* ランダムノイズ点群の作成 */
	cout << "ノイズ点群数：" << noiseNum << endl;
	for( int i = 0; i < noiseNum; i++ ){

		/* ランダムノイズ点座標：入力点群の存在座標範囲内に収まるように作成 */
		temp.x = minX + ( rand() % (int)widthX );	// X
		temp.y = minY + ( rand() % (int)widthY );	// Y
		temp.z = minZ + ( rand() % (int)widthZ );	// Z

		/* ランダムノイズ点色：白 */
		temp.r = 255;
		temp.g = 255;
		temp.b = 255;
		noise1->points.push_back( temp );
	}
	noise1->width = noise1->points.size();
	noise1->height = 1;
	

	/*** 入力点群に対するランダムノイズ点群の散布 ***/

	/* ランダムノイズ散布後の入力点群 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr noisyCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* 入力点群とランダムノイズ点群の統合 */
	*noisyCloud1 = *cloud1 + *noise1;


#ifdef VIEWER
	
	/*** 点群の可視化 ***/
	showPointCloudRGB( noisyCloud1, noiseNum );

#endif

	/*** 点群の出力保存 ***/
	savePointCloudRGBtoPCD( noisyCloud1, outputFileName1 );
	savePointCloudRGBtoPLY( noisyCloud1, outputFileName2 );

	return 0;
}
