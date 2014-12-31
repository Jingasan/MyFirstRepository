/***** 主成分分析( PCA ) *****/


/*** インクルードファイル ***/

/* C++ */
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

/* OpenMP */
#include <omp.h>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

/* ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理関数 ***/
int main( void ){


	/* 処理開始のコール */
	cout << "【 プログラム開始 】" << endl;
	if( INPUTPOINTCLOUDNUM > POINTCLOUDNUM ){ cout << ">> Caution!: INPUTPOINTCLOUDNUM > POINTCLOUDNUM" << endl; return 1; }

	/* 変数の宣言 */
	int maxPointCloudNum = 0;		// 最大点群数
	int minPointCloudNum = INT_MAX;	// 最小点群数
	int maxPointCloudID = -1;		// 最大点群数をとる点群番号
	int minPointCloudID = -1;		// 最小点群数をとる点群番号
	pcl::PointXYZRGB temp;
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr *cloudCluster1[ INPUTPOINTCLOUDNUM ];
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) cloudCluster1[ i ] = new pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* 点群データの読み込み, 最大点群数、最小点群数の取得 */
	cout << ">>> 点群データ( PointXYZ )の読込中..." << "\r";
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ){
		if( pcl::io::loadPCDFile( InputFileName[ i ], **cloudCluster1[ i ] ) == -1 ){ cout << ">>> 点群 " << i << " 読み込み失敗" << endl; return 1; }
		if( cloudCluster1[ i ]->get()->size() > maxPointCloudNum ){ maxPointCloudNum = cloudCluster1[ i ]->get()->size(); maxPointCloudID = i; }
		if( cloudCluster1[ i ]->get()->size() < minPointCloudNum ){ minPointCloudNum = cloudCluster1[ i ]->get()->size(); minPointCloudID = i; }
	}
	cout << ">>> 点群データ( PointXYZ )の読み込み OK" << endl;

#ifdef COMMENT
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) cout << i << ". 最大点群数：" << maxPointCloudNum << "( ID：" << maxPointCloudID << " )" << ", 最小点群数：" << minPointCloudNum << "( ID：" << minPointCloudID << " )" << endl;
#endif


	/*** 点群数の正規化 ***/
	cout << ">>> モデル点群数の正規化中..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr *cloudCluster2[ INPUTPOINTCLOUDNUM ];
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) cloudCluster2[ i ] = new pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* 各モデル点群データの点群数を最小点群数に揃える */
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ){
		for( int j = 0; j < minPointCloudNum; j++ ){
			
			temp.x = cloudCluster1[ i ]->get()->points[ j ].x;
			temp.y = cloudCluster1[ i ]->get()->points[ j ].y;
			temp.z = cloudCluster1[ i ]->get()->points[ j ].z;
			if( ( i % 15 ) == 0 ) temp.r = 255; temp.g = 255; temp.b = 255;
			if( ( i % 15 ) == 1 ) temp.r = 255; temp.g = 0; temp.b = 0;
			if( ( i % 15 ) == 2 ) temp.r = 0; temp.g = 255; temp.b = 0;
			if( ( i % 15 ) == 3 ) temp.r = 0; temp.g = 0; temp.b = 255;
			if( ( i % 15 ) == 4 ) temp.r = 255; temp.g = 255; temp.b = 0;
			if( ( i % 15 ) == 5 ) temp.r = 0; temp.g = 255; temp.b = 255;
			if( ( i % 15 ) == 6 ) temp.r = 255; temp.g = 0; temp.b = 255;
			if( ( i % 15 ) == 7 ) temp.r = 128; temp.g = 128; temp.b = 128;
			if( ( i % 15 ) == 8 ) temp.r = 128; temp.g = 0; temp.b = 0;
			if( ( i % 15 ) == 9 ) temp.r = 0; temp.g = 128; temp.b = 0;
			if( ( i % 15 ) == 10 ) temp.r = 0; temp.g = 0; temp.b = 128;
			if( ( i % 15 ) == 11 ) temp.r = 128; temp.g = 128; temp.b = 0;
			if( ( i % 15 ) == 12 ) temp.r = 0; temp.g = 128; temp.b = 128;
			if( ( i % 15 ) == 13 ) temp.r = 128; temp.g = 0; temp.b = 128;
			if( ( i % 15 ) == 14 ) temp.r = 0; temp.g = 0; temp.b = 0;
			cloudCluster2[ i ]->get()->points.push_back( temp );
		}
		cloudCluster2[ i ]->get()->width = minPointCloudNum;
		cloudCluster2[ i ]->get()->height = 1;
		cout << "ID：" << i << ", 点群数：" << cloudCluster2[ i ]->get()->size() << endl;
	}
	cout << ">>> モデル点群数の正規化 OK" << endl;


#ifdef MERGE

	/* 点群の統合 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr *mergedCloud[ INPUTPOINTCLOUDNUM - 1 ];
	for( int i = 0; i < INPUTPOINTCLOUDNUM - 1; i++ ) mergedCloud[ i ] = new pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );
	for( int i = 0; i < INPUTPOINTCLOUDNUM - 1; i++ ){
		if( i == 0 ) **mergedCloud[ i ] = **cloudCluster2[ i ] + **cloudCluster2[ i + 1 ];
		else **mergedCloud[ i ] = **mergedCloud[ i - 1 ] + **cloudCluster2[ i + 1 ];
	}

	/* 点群の出力保存 */
	savePointCloudRGBtoPLY( *mergedCloud[ INPUTPOINTCLOUDNUM - 2 ], OutputPLYFileName1 );

#endif
#ifdef VIEWER

	/* 点群の表示 */
	cout << ">>> 点群データ( PointXYZRGB )の表示中" << "\r";
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) showPointCloudRGB( *cloudCluster2[ i ], i );
	showPointCloudRGB( *mergedCloud[ INPUTPOINTCLOUDNUM - 2 ], INPUTPOINTCLOUDNUM - 2 );
	cout << ">>> 点群データ( PointXYZRGB )の表示終了" << endl;

#endif


	/*** 主成分分析 ***/

	/* デザイン行列の作成 */
	
	
	const int rowSize = 300;//3 * minPointCloudNum;	// 行の要素数 … モデル点群座標 3 * N 個分 ⇒ x1, y1, z1, x2, y2, z2, …, xN, yN, zN
	const int colSize = INPUTPOINTCLOUDNUM;			// 列の要素数 … モデル数 M 個分 ⇒ モデル1, モデル2, モデル3, …, モデルM
	//Eigen::MatrixXd designMatrix;
	//Eigen::Matrix< double, rowSize, colSize > designMatrix;





	/* 処理終了のコール */
	cout << "【 プログラム終了 】" << endl;

	return 0;
}