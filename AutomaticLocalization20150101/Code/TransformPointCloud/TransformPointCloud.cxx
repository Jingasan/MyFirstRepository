/***** 点群の R, t 変換 *****/
// 入力点群を点群の重心を中心に指定した R, t だけ回転並進

//【 コマンドライン引数 】
// 例1：
// Input/PointData/ModelData/LiverR20/0001.pcd Output/TransformPointCloud/LiverR20/Ra30Rb30Rc30Tx0Ty0Tz0/0001.pcd Output/TransformPointCloud/LiverR20/Ra30Rb30Rc30Tx0Ty0Tz0/0001.ply Output/TransformPointCloud/LiverR20/Ra30Rb30Rc30Tx0Ty0Tz0/Ra30Rb30Rc30Tx0Ty0Tz0.csv 30.0 30.0 30.0 0 0 0
// 例2：
// Input/PointData/ModelData/LiverR20/0001.pcd Output/TransformPointCloud/LiverR20/Ra100Rb100Rc30Tx0Ty0Tz0/0001.pcd Output/TransformPointCloud/LiverR20/Ra100Rb100Rc30Tx0Ty0Tz0/0001.ply Output/TransformPointCloud/LiverR20/Ra100Rb100Rc30Tx0Ty0Tz0/Ra100Rb100Rc30Tx0Ty0Tz0.csv 100.0 100.0 30.0 0 0 0


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
//#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation.h>

/* ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理関数 ***/
int main( int argc, char* argv[] ){


	/* 処理開始のコール */
	cout << "【 プログラム開始 】" << endl;


	/* コマンドライン引数読み込み失敗時 */
	if( argc < 11 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl;
		cerr << " 1. InputPcdFile"<< endl;
		cerr << " 2. OutputPcdFile" << endl;
		cerr << " 3. OutputPlyFile" << endl;
		cerr << " 4. OutputRtFile" << endl;
		cerr << " 5. αdegree" << endl;
		cerr << " 6. βdegree" << endl;
		cerr << " 7. γdegree" << endl;
		cerr << " 8. tx" << endl;
		cerr << " 9. ty" << endl;
		cerr << "10. tz" << endl;
		system( "pause" );
		return -1;
	}

	/* ファイルパスの設定 */
	const char *inputFileName1  = argv[ 1 ];	// 入力点群データ名
	const char *outputFileName1 = argv[ 2 ];	// 出力点群データ名
	const char *outputFileName2 = argv[ 3 ];	// 出力点群データ名
	const char *outputFileName3 = argv[ 4 ];	// 作成した回転行列，並進ベクトルの値を記述したエクセルファイルまでのパス

	/* 変数の宣言 */
	float alphaDegree = atof( argv[ 5 ] );				// α°
	float betaDegree = atof( argv[ 6 ] );				// β°
	float gammaDegree = atof( argv[ 7 ] );				// γ°
	float tx = atof( argv[ 8 ] );						// tx
	float ty = atof( argv[ 9 ] );						// ty
	float tz = atof( argv[ 10 ] );						// tz
	float alpha = ( alphaDegree / 180 ) * PAI;			// α°のラジアン値
	float beta = ( betaDegree / 180 ) * PAI;			// β°のラジアン値
	float gamma = ( gammaDegree / 180 ) * PAI;			// γ°のラジアン値
	Eigen::Matrix3f alphaR;								// αにおける回転行列
	Eigen::Matrix3f betaR;								// βにおける回転行列
	Eigen::Matrix3f gammaR;								// γにおける回転行列
	Eigen::Matrix3f R;									// 回転行列 R
	Eigen::Vector3f t;									// 並進ベクトル t
	Eigen::Matrix4f Rt;									// アフィン変換行列 Rt
	pcl::PointXYZRGB temp;								// 点座標の一時格納用変数
	

	/* 点群情報を格納する構造体変数の定義 */
	PointCloudInfo modelInformation;	// モデル点群
	


	/*** 点群の読み込み：PCD データ ***/
	cout << "【 点群データの読み込み 】" << endl;
	
	/* R, t 変換対象点群データの読み込み */
	cout << ">>> R, t 変換対象点群( PointXYZ )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM( new pcl::PointCloud< pcl::PointXYZ > );	// 入力点群( PointXYZ )のメモリ確保
	if( pcl::io::loadPCDFile( inputFileName1, *cloudM ) == -1 ){ cout << ">>> R, t 変換対象点群データの読み込み失敗" << endl; system( "pause" ); return -1; }
	cout << ">>> R, t 変換対象点群( PointXYZ )の読み込み OK" << endl;



	/*** 入力点群の座標変換 ***/
	cout << "【 入力点群の座標変換(重心を原点座標に設定) 】" << endl;

	/* 座標変換後の点群のメモリ確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// 重心変換後のモデル点群

	/* 初期化 */
	modelInformation.Xmin = DBL_MAX;
	modelInformation.Ymin = DBL_MAX;
	modelInformation.Zmin = DBL_MAX;
	modelInformation.Xmax = -DBL_MAX;
	modelInformation.Ymax = -DBL_MAX;
	modelInformation.Zmax = -DBL_MAX;

	/* モデル点群の端点の算出 */
	for( int i = 0; i < cloudM->points.size(); i++ ){
		
		/* X, Y, Z の最小値を設定 */
		if( cloudM->points[ i ].x < modelInformation.Xmin ) modelInformation.Xmin = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y < modelInformation.Ymin ) modelInformation.Ymin = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z < modelInformation.Zmin ) modelInformation.Zmin = cloudM->points[ i ].z;
		
		/* X, Y, Z の最大値を設定 */
		if( cloudM->points[ i ].x > modelInformation.Xmax ) modelInformation.Xmax = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y > modelInformation.Ymax ) modelInformation.Ymax = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z > modelInformation.Zmax ) modelInformation.Zmax = cloudM->points[ i ].z;
	}

	/* 点群の重心をモデル情報として格納 */
	modelInformation.gravX = ( modelInformation.Xmax + modelInformation.Xmin ) / 2;
	modelInformation.gravY = ( modelInformation.Ymax + modelInformation.Ymin ) / 2;
	modelInformation.gravZ = ( modelInformation.Zmax + modelInformation.Zmin ) / 2;

	/* 点群の重心を原点に設定 */
	modelInformation.Xmin -= modelInformation.gravX; modelInformation.Xmax -= modelInformation.gravX;
	modelInformation.Ymin -= modelInformation.gravY; modelInformation.Ymax -= modelInformation.gravY;
	modelInformation.Zmin -= modelInformation.gravZ; modelInformation.Zmax -= modelInformation.gravZ;

	/* モデル点群 */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* モデル点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp.x = cloudM->points[ i ].x - modelInformation.gravX;
		temp.y = cloudM->points[ i ].y - modelInformation.gravY;
		temp.z = cloudM->points[ i ].z - modelInformation.gravZ;
		temp.r = 255;
		temp.g = 0;
		temp.b = 0;
		modelPointCloud->points.push_back( temp );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;



	/*** 3次元のアフィン変換行列の作成 ***/
	cout << "【 3次元のアフィン変換行列の作成 】" << endl;
	cout << "指定値：" << endl;
	cout << "( α, β, γ, tx, ty, tz ) = ( " << alphaDegree << "°, " << betaDegree << "°, " << gammaDegree << "°, " << tx << ", " << ty << ", " << tz << " )" << endl;
	
	/* オイラー角による回転行列 R の作成 */
	alphaR << cos( alpha ), -sin( alpha ), 0, sin( alpha ), cos( alpha ), 0, 0, 0, 1;
	betaR << cos( beta ), 0, sin( beta ), 0, 1, 0, -sin( beta ), 0, cos( beta );
	gammaR << cos( gamma ), -sin( gamma ), 0, sin( gamma ), cos( gamma ), 0, 0, 0, 1;
	R = alphaR * betaR * gammaR;

	/* 並進ベクトル t の作成 */
	t << tx, ty, tz;

	/* アフィン変換行列 Rt の作成 */
	Rt << R( 0, 0 ), R( 0, 1 ), R( 0, 2 ), t( 0 ), R( 1, 0 ), R( 1, 1 ), R( 1, 2 ), t( 1 ), R( 2, 0 ), R( 2, 1 ), R( 2, 2 ), t( 2 ), 0, 0, 0, 1;

	/* 作成されたアフィン変換行列のコマンドライン表示 */
	cout << ">> Rt：" << endl;
	cout << Rt << endl;
	cout << ">> |R|：" << endl;
	cout << R.determinant() << endl;
	cout << ">> |Rt|：" << endl;
	cout << Rt.determinant() << endl;



	/*** 入力点群のアフィン変換 ***/
	cout << "【 入力点群の R, t 変換 】" << endl;

	/* メモリの確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// 座標変換後の3次元点群( PointXYZ 型を指定 )
	
	/* 4 × 4 の Rt 行列によるアフィン変換 */
	pcl::transformPointCloud( *modelPointCloud, *transformedCloud, Rt );

	/* R, t 変換後の点群色の変更 */
	for( int i = 0; i < transformedCloud->size(); i++ ){
		transformedCloud->points[ i ].r = 0;
		transformedCloud->points[ i ].g = 255;
		transformedCloud->points[ i ].b = 0;
	}



	/*** 2点群の統合と可視化 ***/
#ifdef VIEWER
	cout << "【 変換前の点群と変換後の点群の可視化 】" << endl;
	cout << ">> R, t 変換前の点群 ： RED" << endl;
	cout << ">> R, t 変換後の点群 ： GREEN" << endl;

	/* メモリの確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr merged( new pcl::PointCloud< pcl::PointXYZRGB > );	// 座標変換後の3次元点群( PointXYZ 型を指定 )
	
	/* 統合 */
	*merged = *modelPointCloud + *transformedCloud;

	/* 可視化 */
	showPointCloudRGB( merged, 1 );
#endif


	/*** R, t 変換後の点群の点群を元の座標系に戻す ***/
	cout << "【 R, t 変換後の点群の点群を元の座標系に戻す 】" << endl;

	/* 座標変換後の点群のメモリ確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr finalCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* R, t 変換後の点群の点群を元の座標系に戻す */
	for( int i = 0; i < transformedCloud->size(); i++ ){
			
		/* モデル点群を DF 座標系へ変換：DF 座標系は点群の重心が原点 */
		temp.x = transformedCloud->points[ i ].x + modelInformation.gravX;
		temp.y = transformedCloud->points[ i ].y + modelInformation.gravY;
		temp.z = transformedCloud->points[ i ].z + modelInformation.gravZ;
		temp.r = transformedCloud->points[ i ].r;
		temp.g = transformedCloud->points[ i ].g;
		temp.b = transformedCloud->points[ i ].b;
		finalCloud->points.push_back( temp );
	}
	finalCloud->width = transformedCloud->points.size();
	finalCloud->height = 1;



	/*** 点群の出力保存 ***/
	cout << "【 点群の出力保存 】" << endl;
	savePointCloudRGBtoPCD( finalCloud, outputFileName1 );
	savePointCloudRGBtoPLY( finalCloud, outputFileName2 );


	/*** Rt のエクセル出力保存 ***/
	cout << "【 Rtの出力保存 】" << endl;

	/* ファイルストリーム変数の定義 */
	ofstream output;

	/* 出力ファイルパスの設定 */
	output.open( outputFileName3 );

	/* エクセル出力 */
	output << Rt( 0, 0 ) << "," << Rt( 0, 1 ) << "," << Rt( 0, 2 ) << "," << Rt( 0, 3 ) << endl;
	output << Rt( 1, 0 ) << "," << Rt( 1, 1 ) << "," << Rt( 1, 2 ) << "," << Rt( 1, 3 ) << endl;
	output << Rt( 2, 0 ) << "," << Rt( 2, 1 ) << "," << Rt( 2, 2 ) << "," << Rt( 2, 3 ) << endl;
	output << Rt( 3, 0 ) << "," << Rt( 3, 1 ) << "," << Rt( 3, 2 ) << "," << Rt( 3, 3 ) << endl;

	/* メモリの解放 */
	output.close();


	/* 処理終了のコール */
	cout << "【 プログラム終了 】" << endl;

	return 0;
}