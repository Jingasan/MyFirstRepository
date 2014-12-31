/***** 入力点群に対するガウシアンノイズの付加 *****/

// 【 コマンドライン引数 】
// 指定するコマンドライン引数
// 1. 入力点群ファイル名1.pcd
// 2. 入力点群ファイル名2.pcd
// 3. 出力点群ファイル名3.pcd
// [ 4. ガウシアンの標準偏差幅のスケール値 ]

// 例：
// Input/PointData/ModelData/LiverR20/0001.pcd Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.pcd Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.ply 1


/*** インクルードファイル ***/

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
	

	/* プログラム開始 */
	cout << "【 プログラム開始 】" << endl;

	/* コマンドライン引数読み込み失敗時 */
	if( argc < 4 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << " inputPointCloud.pcd outputPointCloud.pcd outputPointCloud.ply [ NoiseScale ]" << endl;
		return EXIT_FAILURE;
	}

	/* 変数の宣言 */
	float noiseScale = NOISESCALE;					// ガウシアンの標準偏差幅のスケール値の既定値
	const char* inputFileName = argv[ 1 ];			// 入力点群ファイル名
	const char* outputFileName1 = argv[ 2 ];		// 出力点群ファイル名
	const char* outputFileName2 = argv[ 3 ];		// 出力点群ファイル名
	if( argc > 4 ) noiseScale = atof( argv[ 4 ] );	// ガウシアンの標準偏差幅のスケール値
	FILE *inputFp1;									// ガウシアンノイズファイル
	vector< float > gaussianNoiseXYZ;				// ガウシアンノイズの値：x, y, z
	int i = 0;										// ガウシアンノイズ数のカウンタ

	/* メモリの確保 */
	gaussianNoiseXYZ.resize( 3 );	// 配列数： x, y, z のガウシアンノイズ


	/*** ノイズ付加対象点群の読み込み ***/

	/* 入力点群のメモリ確保 */
	cout << ">>> 入力点群( PointXYZRGB )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr inputCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	if( pcl::io::loadPCDFile( inputFileName, *inputCloud ) == -1 ){ cout << ">>> 入力点群データの読み込み失敗" << endl; return 1; }
	cout << ">>> 入力点群( PointXYZRGB )の読み込み OK" << endl;


	/*** ガウシアンノイズ付加点群の作成 ***/

	/* ガウシアンノイズ付加点群のメモリ確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr gaussianNoisedCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointXYZRGB temp;

	/* ガウシアンノイズファイルの展開 */
	if( ( inputFp1 = fopen( InputGaussianNoiseFileName, "r" ) ) == NULL ){ cout << "Caution!: " << InputGaussianNoiseFileName << " open error" << endl; return 1; }

	/* ファイル内容の読み込みと入力点座標に対するガウシアンノイズの付加 */
	cout << "----- ガウシアンノイズの付加 -----" << endl;
	cout << ">> The number of Input Points：" << inputCloud->size() << endl;
	while( fscanf( inputFp1, "%f,%f,%f", &gaussianNoiseXYZ[ 0 ], &gaussianNoiseXYZ[ 1 ], &gaussianNoiseXYZ[ 2 ] ) != EOF ){

		temp.x = inputCloud->points[ i ].x + noiseScale * gaussianNoiseXYZ[ 0 ];
		temp.y = inputCloud->points[ i ].y + noiseScale * gaussianNoiseXYZ[ 1 ];
		temp.z = inputCloud->points[ i ].z + noiseScale * gaussianNoiseXYZ[ 2 ];
		temp.r = 255;
		temp.g = 255;
		temp.b = 0;
		gaussianNoisedCloud->points.push_back( temp );

#ifdef COMMENT
		if( i % 100 == 0 ){
			cout << ">> Point Number  ：" << i << endl;
			cout << ">> Gaussian Noise：" << noiseScale * gaussianNoiseXYZ[ 0 ] << " " << noiseScale * gaussianNoiseXYZ[ 1 ] << " " << noiseScale * gaussianNoiseXYZ[ 2 ] << endl;
			cout << ">> Input Point   ：" << inputCloud->points[ i ].x << " " << inputCloud->points[ i ].y << " " << inputCloud->points[ i ].z << endl;
			cout << ">> Output Point  ：" << gaussianNoisedCloud->points[ i ].x << " " << gaussianNoisedCloud->points[ i ].y << " " << gaussianNoisedCloud->points[ i ].z << endl;
		}
#endif
		i++;
		if( i == inputCloud->size() ) break;	// 入力点群すべてにガウシアンノイズが付加された場合
	}
	gaussianNoisedCloud->width = gaussianNoisedCloud->points.size();
	gaussianNoisedCloud->height = 1;

	/* 入力ガウシアンノイズの数がノイズ付加対象の点群数よりも少ない場合 */
	if( i < inputCloud->size() ){
		cout << "Caution!：The number of Gaussian noise points is fewer than the input points." << endl;
		cout << "The number of Gaussian Noise：" << i << endl;
		return 1;
	}

	/* メモリの解放 */
	fclose( inputFp1 );


#ifdef VIEWER
	
	/*** 点群の可視化 ***/
	cout << ">> 入力点群表示中" << "\r";
	showPointCloudRGB( inputCloud, inputCloud->size() );
	cout << ">> 入力点群表示終了" << endl;
	cout << ">> ガウシアンノイズ付加点群表示中" << "\r";
	showPointCloudRGB( gaussianNoisedCloud, i );
	cout << ">> ガウシアンノイズ付加点群表示終了" << endl;

#endif

	/*** 点群の出力保存 ***/
	savePointCloudRGBtoPCD( gaussianNoisedCloud, outputFileName1 );
	savePointCloudRGBtoPLY( gaussianNoisedCloud, outputFileName2 );

	/* プログラム終了 */
	cout << "【 プログラム終了 】" << endl;

	return 0;
}
