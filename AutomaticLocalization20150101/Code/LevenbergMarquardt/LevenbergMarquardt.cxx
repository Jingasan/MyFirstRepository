/***** Levenberg Marquardt 法のオフライン版 *****/
// モデル点群と探索対象点群の2つと、全探索により得られたR,tのデータを読み込み、最適化による詳細な位置合わせを行う

// 【 コマンドライン引数 】
// 1. 探索対象点群ファイル名.pcd
// 2. モデル点群ファイル名.pcd
// 3. 全探索結果のR,tのファイル名.csv

// 例1：
// Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd Output/ExhaustiveSearchResult/SearchingParameters.csv


/*** インクルード ***/

/* C */
#include <math.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <limits.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <utility>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>			// KdTree
#include <pcl/kdtree/kdtree_flann.h>	// KdTree

/* 設定ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理関数 ***/
int main( int argc, char **argv ){


	/* 変数の宣言 */
	FILE *inputFile;						// 入力ファイル
	const char *inputFileName1 = argv[ 1 ];	// 探索対象点群ファイル名
	const char *inputFileName2 = argv[ 2 ];	// モデル点群ファイル名
	const char *inputFileName3 = argv[ 3 ];	// 全探索結果 R, t のファイル名
	Eigen::Matrix< double, 3, 3 > ER;		// 全探索結果の回転行列
	Eigen::Vector3d Et;						// 全探索結果の並進ベクトル
	float inp1, inp2, inp3, inp4;			// 一時格納用変数
	int n = 0;								// カウンタ変数

	/* 点群情報を格納する構造体変数の定義 */
	PointCloudInfo modelInformation;	// モデル点群
	PointCloudInfo targetInformation;	// 探索対象点群



	/*** プログラム開始のコール ***/
	cout << "【 プログラム開始 】" << endl;



	/*** 点群の読み込み：PCD データ ***/
	cout << "【 点群のデータの読み込み 】" << endl;
	
	/* 探索対象点群データの読み込み */
	cout << ">>> 探索対象点群( PointXYZ )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT( new pcl::PointCloud< pcl::PointXYZ > );	// 入力点群( PointXYZ )のメモリ確保
	if( pcl::io::loadPCDFile( inputFileName1, *cloudT ) == -1 ){ cout << ">>> 探索対象点群データの読み込み失敗" << endl; return 1; }
	cout << ">>> 探索対象点群( PointXYZ )の読み込み OK" << endl;

	/* モデル点群データの読み込み */
	cout << ">>> モデル点群( PointXYZ )の読込中..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM( new pcl::PointCloud< pcl::PointXYZ > );	// 入力点群( PointXYZ )のメモリ確保
	if( pcl::io::loadPCDFile( inputFileName2, *cloudM ) == -1 ){ cout << ">>> モデル点群データの読み込み失敗" << endl;return 1; }
	cout << ">>> モデル点群( PointXYZ )の読み込み OK" << endl;



	/*** 全探索結果の読み込み ***/
	cout << "【 全探索結果の R, t の読み込み 】" << endl;
	cout << ">>> 全探索結果の読込中..." << "\r";
	
	/* ファイル読み込み失敗時 */
	if( ( inputFile = fopen( inputFileName3, "r" ) ) == NULL ){
		cout << "全探索結果データの読み込み失敗" << endl;
		exit( 0 );
	}

	/* ファイル内容の読み込み */
	while( 1 ){
		if( n == 0 ) fscanf( inputFile, "%c,,,%c,%c,%c", &inp1, &inp2, &inp3, &inp4 );				// 1行目部分は読み飛ばす
		else if( n > 0 ){
			fscanf( inputFile, "%f,%f,%f,%f", &inp1, &inp2, &inp3, &inp4 );							// 2行目以下から読み込み
			ER( n - 1, 0 ) = inp1; ER( n - 1, 1 ) = inp2; ER( n - 1, 2 ) = inp3; Et( n - 1 ) = inp4;	// ファイル内容を R, t に格納
		}
		if( n == 3 ) break;
		n++;
	}

	/* メモリの解放 */
	fclose( inputFile );
	cout << ">>> 全探索結果の読み込み OK" << endl;
	


	/*** 点群の座標変換 ***/
	cout << "【 点群の座標変換 】" << endl;
	cout << ">> 点群の重心を原点座標に設定" << endl;

	/* 座標変換後の点群のメモリ確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// 重心変換後の探索対象点群
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// 重心変換後のモデル点群

	/* 点群の座標変換 */
	CoordinateTransformation(
		&modelInformation,	// モデル点群情報
		&targetInformation,	// 探索対象点群情報
		cloudM,				// 座標変換前のモデル点群
		cloudT,				// 座標変換前の探索対象点群
		modelPointCloud,	// 座標変換後のモデル点群
		targetPointCloud	// 座標変換後の探索対象点群
	);



	/*** マーカート法による最適化 ***/
	cout << "【 マーカート法による最適化 】" << endl;

	/* 変数の宣言 */
	double C = 0;							// 現在の評価値 C
	double nextC = 0;						// 次の評価値 C
	double absC = DBL_MAX;					// 現在の評価値 C と次の評価値 C の差の絶対値：| nextC - C |
	double absA = DBL_MAX;					// 現在のパラメータベクトル A と次のパラメータベクトル A の差の絶対値：| nextA - A |
	double lambda = LAMBDA;					// マーカート法の係数λ
	int loopCounter = 0;					// ループ回数
	Eigen::VectorXd A( 6 );					// 現在のパラメータベクトル：A = [ r1, r2, r3, t1, t2, t3 ]T
	Eigen::VectorXd nextA( 6 );				// 次のパラメータベクトル：nextA = [ r1, r2, r3, t1, t2, t3 ]T
	Eigen::VectorXd difA( 6 );				// 現在のパラメータベクトル A と次のパラメータベクトル A の差
	Eigen::VectorXd dfA( 6 );				// 評価関数 C の一階微分：dfA = [ ∂C/∂r1, ∂C/r2, ∂C/r3, ∂C/∂t1, ∂C/∂t2, ∂C/∂t3 ]T
	Eigen::Matrix< double, 3, 3 > initR;	// マーカート法の R の初期位置
	Eigen::Vector3d initT;					// マーカート法の t の初期位置
	Eigen::Matrix< double, 3, 3 > R;		// R
	Eigen::Vector3d t;						// t
	Eigen::Matrix< double, 3, 3 > nextR;	// nextR
	Eigen::Vector3d nextT;					// nextT
	Eigen::Matrix< double, 6, 6 > H;		// 6 × 6 ヘッセ行列
	Eigen::Matrix< double, 6, 6 > I;		// 6 × 6 単位行列
	Eigen::Vector3d Rodrigues;				// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]T
	Eigen::Matrix< double, 3, 3 > dR[ 3 ];	// r1, r2, r3 それぞれによる回転行列の偏微分結果の 3 × 3 行列
	vector< int > index( 1 );				// 最近点のインデックス
	vector< float > distance2( 1 );			// 最近点までの距離の二乗
	pcl::PointXYZRGB transformedModelPoint;	// R, t による変換後のモデル点座標
	Eigen::Vector3d ap;						// R, t による変換後のモデル点座標
	Eigen::Vector3d bp;						// R, t による変換前のモデル点座標
	Eigen::Vector3d Yi;						// Yi
	Eigen::Vector3d tYi;					// t - Yi
	Eigen::Vector3d Xi;						// Xi
	Eigen::Vector3d RXitYi;					// R * Xi + t - Yi
	Eigen::Vector3d YiRXit;					// Yi - ( R * Xi + t )
	double norm;							// || Yi - ( R * Xi + t ) ||
	double norm2;							// || Yi - ( R * Xi + t ) || の2乗
	Eigen::VectorXd jacob( 6 );				// ヤコビアン
	Eigen::Matrix< double, 6, 6 > jacob2;	// jacob * jacobT
	Eigen::Matrix< double, 6, 6 > lambdaI;	// λI
	Eigen::Matrix< double, 6, 6 > lambdaIH;	// λI + H
	Eigen::Matrix< double, 6, 6 > lambdaIHinv;	// ( λI + H )-1
	int flag = 0;

	/* KdTree の作成 */
	pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree;	// KdTree クラスのオブジェクト変数の宣言
	kdtree.setInputCloud( targetPointCloud );		// KdTree に対して点群をセット

	/* R と t の初期値の設定 */
	initR = ER;	// 全探索結果の R を初期値に設定
	initT = Et;	// 全探索結果の t を初期値に設定

	/* 単位行列の作成 */
	I = Eigen::MatrixXd::Identity( 6, 6 );	// 単位行列の作成
	H = Eigen::MatrixXd::Zero( 6, 6 );		// ヘッセ行列の全要素を 0 に初期化
	dfA = Eigen::VectorXd::Zero( 6 );		// 評価関数 C の一階微分ベクトルの全要素を 0 に初期化

	/* 初期パラメータベクトル A の算出 */

	/* Rodrigues の回転ベクトル r = [ r1, r2, r3 ]T の作成 */
	Rodrigues = ConvertRodrigues( &initR );

	/* 回転行列の偏微分 */
	PartialDerivativeForRodrigues( &Rodrigues, dR );	// 回転の３要素 r1, r2, r3 で微分した結果を取得し、dR[ 3 ] に格納

	/* R, t の設定 */
	R = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues の回転ベクトルからの回転行列を復元
	t = initT;

	/* パラメータベクトル A の設定 */
	A( 0 ) = Rodrigues( 0 );	// r1
	A( 1 ) = Rodrigues( 1 );	// r2
	A( 2 ) = Rodrigues( 2 );	// r3
	A( 3 ) = t( 0 );			// t1
	A( 4 ) = t( 1 );			// t2
	A( 5 ) = t( 2 );			// t3


	/* 初期の評価値 C の算出 */
	C = 0;
	for( int i = 0; i < modelPointCloud->points.size(); i++ ){

		/* R, t によるモデル点座標の変換 */
		bp( 0 ) = modelPointCloud->points[ i ].x;
		bp( 1 ) = modelPointCloud->points[ i ].y;
		bp( 2 ) = modelPointCloud->points[ i ].z;
		ap = R * bp + t;
		transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
		transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
		transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

		/* KdTree による対応点探索 */
		kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// モデル点群の各点から探索対象点群中の最近点までの距離の算出

		/* 評価値 C の算出 */
		C += distance2[ 0 ];

		/* Yi と Xi */
		Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
		Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
		Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

		/* t - Yi */
		tYi = t - Yi;

		/* RXi + t - Yi */
		RXitYi = R * Xi + t - Yi;

		/* 勾配ベクトル */
		jacob( 0 ) = tYi.transpose() * dR[ 0 ] * Xi;
		jacob( 1 ) = tYi.transpose() * dR[ 1 ] * Xi;
		jacob( 2 ) = tYi.transpose() * dR[ 2 ] * Xi;
		jacob( 3 ) = RXitYi( 0 );
		jacob( 4 ) = RXitYi( 1 );
		jacob( 5 ) = RXitYi( 2 );

		/* 評価関数 C の一階微分 */
		dfA += jacob;	// dfA = [ ∂C/∂r1, ∂C/r2, ∂C/r3, ∂C/∂t1, ∂C/∂t2, ∂C/∂t3 ]T
	}


	/* 初期のヘッセ行列の算出 */
	H = dfA * dfA.transpose();


	/* 初期の次のパラメータベクトルの算出 */
	lambdaI = lambda * I.array();		// λI
	lambdaIH = lambdaI + H;				// λI + H
	lambdaIHinv = lambdaIH.inverse();	// ( λI + H )-1
	nextA = A - lambdaIHinv * dfA;		// 次のパラメータベクトル A


	/* 初期の次の回転行列、並進ベクトルの算出 */
	Rodrigues( 0 ) = nextA( 0 );	// r1
	Rodrigues( 1 ) = nextA( 1 );	// r2
	Rodrigues( 2 ) = nextA( 2 );	// r3
	nextT( 0 ) = nextA( 3 );		// t1
	nextT( 1 ) = nextA( 4 );		// t2
	nextT( 2 ) = nextA( 5 );		// t3
	nextR = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues の回転ベクトルからの回転行列 R を復元


	/* 初期の次の評価値 C の算出 */
	nextC = 0;
	for( int i = 0; i < modelPointCloud->points.size(); i++ ){

		/* R, t によるモデル点座標の変換 */
		bp( 0 ) = modelPointCloud->points[ i ].x;
		bp( 1 ) = modelPointCloud->points[ i ].y;
		bp( 2 ) = modelPointCloud->points[ i ].z;
		ap = R * bp + t;
		transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
		transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
		transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

		/* KdTree による対応点探索 */
		kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// モデル点群の各点から探索対象点群中の最近点までの距離の算出

		/* Yi と Xi */
		Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
		Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
		Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

		/* 次の評価値 C の算出 */
		YiRXit = Yi - ( nextR * Xi + nextT );	// Yi - ( R * Xi + t )
		norm = YiRXit.norm();					// || Yi - ( R * Xi + t ) ||
		norm2 = pow( norm, 2 );					// || Yi - ( R * Xi + t ) || の2乗
		nextC += norm2;							// 次の評価値 C
	}


	cout << "InitC: " << endl << C << endl;
	cout << "A: " << endl << A << endl;
	cout << "InitNextC: " << endl << nextC << endl;
	cout << "nextA: " << endl << nextA << endl;


	/* マーカート法による最適化のループ */
	while( 1 ){


		/* ループ回数のコマンドライン出力 */
		cout << "---------- " << loopCounter + 1 << " ----------" << endl;

		
		/* 次の評価値 C が改善された場合 */
		if( nextC < C ){

			/* 初期化 */
			H = Eigen::MatrixXd::Zero( 6, 6 );		// ヘッセ行列の全要素を 0 に初期化
			dfA = Eigen::VectorXd::Zero( 6 );		// 評価関数 C の一階微分ベクトルの全要素を 0 に初期化

			absC = fabs( nextC - C );			// 現在の評価値 C と次の評価値 C の差の絶対値：| nextC - C |
			difA = nextA - A;
			norm = difA.norm();
			absA = fabs( norm );

			cout << "C: " << endl << C << endl;
			cout << "A: " << endl << A << endl;
			cout << "NextC: " << endl << nextC << endl;
			cout << "NextA: " << endl << nextA << endl;

			if( absC < THRESHOLD_C ) break;		// 評価値 C の収束による最適化終了
			if( absA < THRESHOLD_A ) break;		// パラメータベクトル A の収束による最適化終了

			/* 更新 */
			C = nextC;							// 現在の評価値 C を更新
			A = nextA;							// 現在のパラメータベクトル A を更新
			R = nextR;							// 現在の回転行列の更新
			t = nextT;							// 現在の並進ベクトルの更新
			lambda = lambda * 0.1;				// 係数λの値を 0.1 倍：ニュートン法の影響増大化

			for( int i = 0; i < modelPointCloud->points.size(); i++ ){

				/* R, t によるモデル点座標の変換 */
				bp( 0 ) = modelPointCloud->points[ i ].x;
				bp( 1 ) = modelPointCloud->points[ i ].y;
				bp( 2 ) = modelPointCloud->points[ i ].z;
				ap = R * bp + t;
				transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
				transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
				transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

				/* KdTree による対応点探索 */
				kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// モデル点群の各点から探索対象点群中の最近点までの距離の算出

				/* Yi と Xi */
				Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
				Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
				Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;
				
				/* t - Yi */
				tYi = t - Yi;

				/* RXi + t - Yi */
				RXitYi = R * Xi + t - Yi;

				/* 勾配ベクトル */
				jacob( 0 ) = tYi.transpose() * dR[ 0 ] * Xi;
				jacob( 1 ) = tYi.transpose() * dR[ 1 ] * Xi;
				jacob( 2 ) = tYi.transpose() * dR[ 2 ] * Xi;
				jacob( 3 ) = RXitYi( 0 );
				jacob( 4 ) = RXitYi( 1 );
				jacob( 5 ) = RXitYi( 2 );

				/* 評価関数 C の一階微分 */
				dfA += jacob;	// dfA = [ ∂C/∂r1, ∂C/r2, ∂C/r3, ∂C/∂t1, ∂C/∂t2, ∂C/∂t3 ]T
			}

			/* ヘッセ行列 */
			H = dfA * dfA.transpose();

			/* 次のパラメータベクトルの算出 */
			lambdaI = lambda * I.array();		// λI
			lambdaIH = lambdaI + H;				// λI + H
			lambdaIHinv = lambdaIH.inverse();	// ( λI + H )-1
			nextA = A - lambdaIHinv * dfA;		// 次のパラメータベクトル A
		}
		
		
		/* 次の評価値 C が改悪された場合 */
		else if( nextC >= C ){

			/* λの更新 */
			lambda = lambda * 10;

			/* 次のパラメータベクトルの算出 */
			lambdaI = lambda * I.array();		// λI
			lambdaIH = lambdaI + H;				// λI + H
			lambdaIHinv = lambdaIH.inverse();	// ( λI + H )-1
			nextA = A - lambdaIHinv * dfA;		// 次のパラメータベクトル A
		}

		cout << "λ: " << endl << lambda << endl;

		/* nextR, nextT の取得 */
		Rodrigues( 0 ) = nextA( 0 );
		Rodrigues( 1 ) = nextA( 1 );
		Rodrigues( 2 ) = nextA( 2 );
		PartialDerivativeForRodrigues( &Rodrigues, dR );	// 回転の３要素 r1, r2, r3 で微分した結果を取得し、dR[ 3 ] に格納
		nextR = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues の回転ベクトルからの回転行列を復元
		nextT( 0 ) = nextA( 3 );
		nextT( 1 ) = nextA( 4 );
		nextT( 2 ) = nextA( 5 );


		/* 次の評価値 C の算出 */
		nextC = 0;
		for( int i = 0; i < modelPointCloud->points.size(); i++ ){

			/* R, t によるモデル点座標の変換 */
			bp( 0 ) = modelPointCloud->points[ i ].x;
			bp( 1 ) = modelPointCloud->points[ i ].y;
			bp( 2 ) = modelPointCloud->points[ i ].z;
			ap = R * bp + t;
			transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
			transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
			transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

			/* KdTree による対応点探索 */
			kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// モデル点群の各点から探索対象点群中の最近点までの距離の算出

			/* Yi と Xi */
			Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
			Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
			Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

			/* 次の評価値 C の算出 */
			YiRXit = Yi - ( nextR * Xi + nextT );	// Yi - ( R * Xi + t )
			norm = YiRXit.norm();					// || Yi - ( R * Xi + t ) ||
			norm2 = pow( norm, 2 );					// || Yi - ( R * Xi + t ) || の2乗
			nextC += norm2;							// 次の評価値 C
		}


		/* ループ回数の更新 */
		loopCounter++;

		if( loopCounter >= LOOP_MAX ) break;	// ループ回数の閾値による最適化終了
	}





















#ifdef CUT

	/* マーカート法による最適化のループ */
	while( 1 ){

		/* ループ回数のコマンドライン出力 */
		cout << loopCounter + 1 << endl;

		/* ループ初回時 */
		if( loopCounter == 0 ){

			/* Rodrigues の回転ベクトル r = [ r1, r2, r3 ]T の作成 */
			Rodrigues = ConvertRodrigues( &initR );

			/* 回転行列の偏微分 */
			PartialDerivativeForRodrigues( &Rodrigues, dR );	// 回転の３要素 r1, r2, r3 で微分した結果を取得し、dR[ 3 ] に格納

			/* R, t の設定 */
			R = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues の回転ベクトルからの回転行列を復元
			t = initT;

			/* パラメータベクトル A の設定 */
			A( 0 ) = Rodrigues( 0 );	// r1
			A( 1 ) = Rodrigues( 1 );	// r2
			A( 2 ) = Rodrigues( 2 );	// r3
			A( 3 ) = t( 0 );			// t1
			A( 4 ) = t( 1 );			// t2
			A( 5 ) = t( 2 );			// t3
		}

		/* ループ非初回時 */
		if( loopCounter > 0 ){

			/* マーカート法による最適化後の点群 */
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedModelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// 変換されたモデル点群
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );			// 統合された２点群

			/* 次の r1, r2, r3, t1, t2, t3 の取得 */
			Rodrigues( 0 ) = nextA( 0 );	// r1
			Rodrigues( 1 ) = nextA( 1 );	// r2
			Rodrigues( 2 ) = nextA( 2 );	// r3
			nextT( 0 ) = nextA( 3 );		// t1
			nextT( 1 ) = nextA( 4 );		// t2
			nextT( 2 ) = nextA( 5 );		// t3

			/* Rodrigues の回転ベクトルからの回転行列 R を復元 */
			nextR = ReconstructRFromRodrigues( &Rodrigues );	// R
			nextC = 0;	// 次の評価値 nextC を初期化

			for( int i = 0; i < modelPointCloud->points.size(); i++ ){

				/* R, t によるモデル点座標の変換 */
				bp( 0 ) = modelPointCloud->points[ i ].x;
				bp( 1 ) = modelPointCloud->points[ i ].y;
				bp( 2 ) = modelPointCloud->points[ i ].z;
				ap = R * bp + t;
				transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
				transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
				transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

				/* KdTree による対応点探索 */
				kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// モデル点群の各点から探索対象点群中の最近点までの距離の算出

				/* Yi と Xi */
				Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
				Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
				Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

				/* 次の評価値 C の算出 */
				YiRXit = Yi - ( nextR * Xi + nextT );	// Yi - ( R * Xi + t )
				norm = YiRXit.norm();					// || Yi - ( R * Xi + t ) ||
				norm2 = pow( norm, 2 );					// || Yi - ( R * Xi + t ) || の2乗
				nextC += norm2;							// 次の評価値 C

				/* R, t による変換後の点群の作成 */
				transformedModelPointCloud->points.push_back( transformedModelPoint );
			}

			transformedModelPointCloud->width = modelPointCloud->points.size();
			transformedModelPointCloud->height = 1;

			/* 2点群の統合 */
			*mergedPointCloud = *transformedModelPointCloud + *targetPointCloud;

			/* 2点群の出力 */
			if( pcl::io::savePLYFileASCII( OutputFileName, *mergedPointCloud ) == -1 ){ cout << "Caution!: PCD データの書き込み失敗" << endl; }
		}
		
		/* 次の評価値 C が現在の評価値 C よりも小さくなった場合 or ループの初回時 */
		if( nextC < C || loopCounter == 0 ){

			/* 評価値 C, パラメータベクトル A, 係数λの更新 */
			if( loopCounter > 0 ){
				
				absC = fabs( nextC - C );			// 現在の評価値 C と次の評価値 C の差の絶対値：| nextC - C |
				difA = nextA - A;
				norm = difA.norm();
				absA = fabs( norm );

				cout << "C: " << C << endl;
				cout << "nextC: " << nextC << endl;
				cout << "absC: " << absC << endl;
				cout << "absA: " << absA << endl;

				if( absC < THRESHOLD_C ) break;		// 評価値 C の収束による最適化終了
				if( absA < THRESHOLD_A ) break;		// パラメータベクトル A の収束による最適化終了

				C = nextC;							// 現在の評価値 C を更新
				A = nextA;							// 現在のパラメータベクトル A を更新
				R = nextR;							// 現在の回転行列の更新
				t = nextT;							// 現在の並進ベクトルの更新
				//nextA = Eigen::VectorXd::Zero( 6 );	// 次のパラメータベクトル nextA を初期化
				lambda = lambda * 0.1;				// 係数λの値を 0.1 倍：ニュートン法の影響増大化
				flag = 1;
			}

			/* 初期化 */
			H = Eigen::MatrixXd::Zero( 6, 6 );		// ヘッセ行列の全要素を 0 に初期化
			I = Eigen::MatrixXd::Identity( 6, 6 );	// 単位行列の作成
			dfA = Eigen::VectorXd::Zero( 6 );		// 評価関数 C の一階微分ベクトルの全要素を 0 に初期化
		
			/* 評価関数 C の一階微分の算出 */
			for( int i = 0; i < modelPointCloud->points.size(); i++ ){

				/* R, t によるモデル点座標の変換 */
				bp( 0 ) = modelPointCloud->points[ i ].x;
				bp( 1 ) = modelPointCloud->points[ i ].y;
				bp( 2 ) = modelPointCloud->points[ i ].z;
				ap = R * bp + t;
				transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
				transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
				transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

				/* KdTree による対応点探索 */
				kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// モデル点群の各点から探索対象点群中の最近点までの距離の算出

				/* Yi と Xi */
				Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
				Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
				Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

				/* 初回時における評価値 C の算出 */
				if( loopCounter == 0 ){
					YiRXit = Yi - ( R * Xi + t );	// Yi - ( R * Xi + t )
					norm = YiRXit.norm();			// || Yi - ( R * Xi + t ) ||
					norm2 = norm * norm;			// || Yi - ( R * Xi + t ) || の2乗
					C += norm2;						// 評価関数 C
				}

				/* t - Yi */
				tYi = t - Yi;

				/* RXi + t - Yi */
				RXitYi = R * Xi + t - Yi;

				/* 勾配ベクトル */
				jacob( 0 ) = tYi.transpose() * dR[ 0 ] * Xi;
				jacob( 1 ) = tYi.transpose() * dR[ 1 ] * Xi;
				jacob( 2 ) = tYi.transpose() * dR[ 2 ] * Xi;
				jacob( 3 ) = RXitYi( 0 );
				jacob( 4 ) = RXitYi( 1 );
				jacob( 5 ) = RXitYi( 2 );

				/* 評価関数 C の一階微分 */
				dfA += jacob;	// dfA = [ ∂C/∂r1, ∂C/r2, ∂C/r3, ∂C/∂t1, ∂C/∂t2, ∂C/∂t3 ]T

				/* Jacob * JacobT */
				//jacob2 = jacob * jacob.transpose();

				/* ヘッセ行列 */
				//H += jacob2;
			}

			/* ヘッセ行列の算出 */
			H = dfA * dfA.transpose();

			/* 次のパラメータベクトルの算出 */
			lambdaI = lambda * I.array();		// λI
			lambdaIH = lambdaI + H;				// λI + H
			lambdaIHinv = lambdaIH.inverse();	// ( λI + H )-1
			nextA = A - lambdaIHinv * dfA;		// 次のパラメータベクトル A
		}
		
		cout << "C: " << C << endl;
		cout << "nextC: " << nextC << endl;

		/* 次の評価値 C が現在の評価値 C よりも大きくなった場合 & ループ非初回時 */
		if( nextC >= C && loopCounter > 0 && flag == 0 ){

			/* λの値を小さく変更 */
			lambda = lambda * 10;	// 係数λの値を 10 倍：勾配法の影響増大化

			/* 次のパラメータベクトルの算出 */
			lambdaI = lambda * I.array();		// λI
			lambdaIH = lambdaI + H;				// λI + H
			lambdaIHinv = lambdaIH.inverse();	// ( λI + H )-1
			nextA = A - lambdaIHinv * dfA;		// 次のパラメータベクトル A
		}

		cout << "λ: " << lambda << endl;
		
		flag = 0;

		/* ループカウンタの更新 */
		loopCounter++;

		if( loopCounter >= LOOP_MAX ) break;	// ループ回数の閾値による最適化終了
	}
#endif


	/*** プログラム終了のコール ***/
	cout << "【 プログラム終了 】" << endl;


	return 0;
}