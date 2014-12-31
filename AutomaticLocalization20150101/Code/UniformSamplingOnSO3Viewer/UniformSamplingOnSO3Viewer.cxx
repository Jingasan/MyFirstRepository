/***** 回転空間の均等なサンプリングの可視化 *****/


/*** フラグの定義 ***/
//#define COMMENT	// 途中結果のコマンドライン出力 ( ON : 実行, OFF : 無視 )


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
#include <CodeAnalysis\Warnings.h>

/* OpenMP */
#include <omp.h>

/* PCL 1.6.0 */
#pragma warning( push )
#pragma warning( disable : ALL_CODE_ANALYSIS_WARNINGS )
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#pragma warning( pop )


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 定数の定義 ***/
char input3RadianFileName[ 64 ] = "Output/SamplingR/36864radian.csv";
char outputSamplingS2HopfPCDFileName[ 64 ] = "Output/SamplingR/samplingS2Hopf.pcd";
char outputSamplingS2HopfPLYFileName[ 64 ] = "Output/SamplingR/samplingS2Hopf.ply";
char outputSamplingS2HEALPixPCDFileName[ 64 ] = "Output/SamplingR/samplingS2HEALPix.pcd";
char outputSamplingS2HEALPixPLYFileName[ 64 ] = "Output/SamplingR/samplingS2HEALPix.ply";


/*** メイン処理関数 ***/
int main( void ){


	/* 処理開始のコール */
	cout << "【 プログラム開始 】" << endl;
	

	/* 変数の定義 */
	FILE *inputFile;						// ファイル型変数
	vector< vector< float > > radians;		// 回転角 θ, Φ, Ψ ( 単位：ラジアン ) の格納用行列
	vector< float > radian;					// 回転角 θ, Φ, Ψ ( 単位：ラジアン ) の格納用ベクトル
	vector< vector< float > > radiansS2;	// 球面座標 S2 上の回転角 θ, Φ ( 単位：ラジアン ) の格納用行列
	vector< float > radianS2;				// 球面座標 S2 上の回転角 θ, Φ ( 単位：ラジアン ) の格納用ベクトル
	int count = 0;	// カウンタ変数
	

	/*** エクセルファイルの読み込み ***/

	/* ファイル読み込み失敗時 */
	if( ( inputFile = fopen( input3RadianFileName, "r" ) ) == NULL ){
		cout << "ファイル " << input3RadianFileName << " が開けません" << endl;
		exit( 0 );
	}
	
	/* メモリの確保 */
	radians.resize( 0 );
	radiansS2.resize( 0 );
	radian.resize( 3 );
	radianS2.resize( 2 );
	
	/* ファイル内容の読み込み */
	while( fscanf( inputFile, "%f,%f,%f", &radian[ 0 ], &radian[ 1 ], &radian[ 2 ] ) != EOF ){

		/* 2次元配列への1次元配列の追加 */
		radians.push_back( radian );	// ベクトルの末尾に第1引数を追加する
		if( ( count % 6 ) == 0 ){
			radianS2[ 0 ] = radian[ 0 ], radianS2[ 1 ] = radian[ 1 ];	// 回転角θとΦのみの取得
			radiansS2.push_back( radianS2 );	// ベクトルの末尾に第1引数を追加する
		}
		count++;
	}
	
	/* メモリの解放 */
	fclose( inputFile );
	

#ifdef COMMENT

	/* 回転角 θ, Φ, Ψ ( 単位：ラジアン ) のコマンドライン出力 */
	cout << "【 回転角 θ, Φ, Ψ ( 単位：ラジアン ) 】" << endl;
	for( int i = 0; i < radians.size(); i++ ){
		for( int j = 0; j < radians[ i ].size(); j++ ){
			cout << radians[ i ][ j ] << " ";
		}
		cout << endl;
	}

	/* 回転角 θ, Φ ( 単位：ラジアン ) のコマンドライン出力 */
	cout << "【 回転角 θ, Φ ( 単位：ラジアン ) 】" << endl;
	for( int i = 0; i < radiansS2.size(); i++ ){
		for( int j = 0; j < radiansS2[ i ].size(); j++ ){
			cout << radiansS2[ i ][ j ] << " ";
		}
		cout << endl;
	}

#endif // COMMENT


	/*** 球面座標S2上の3次元点群の作成 ( Hopf座標表現 ) ***/
	cout << "【 球面座標S2上の3次元点群の可視化 ( Hopf座標表現 ) 】" << endl;

	/* 3次元点群の作成 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampS2Hopf( new pcl::PointCloud<pcl::PointXYZRGB> );
	pcl::PointXYZRGB temp1;
	for( int i = 0; i < radiansS2.size(); i++ ){
		temp1.x = cos( radiansS2[ i ][ 0 ] / 2 );
		temp1.y = sin( radiansS2[ i ][ 0 ] / 2 ) * sin( radiansS2[ i ][ 1 ] );
		temp1.z = sin( radiansS2[ i ][ 0 ] / 2 ) * cos( radiansS2[ i ][ 1 ] );
		temp1.r = 255;
		temp1.g = 0;
		temp1.b = 255;
		sampS2Hopf->points.push_back( temp1 );
		//temp1.x = - cos( radiansS2[ i ][ 0 ] / 2 );
		//temp1.y = - sin( radiansS2[ i ][ 0 ] / 2 ) * sin( radiansS2[ i ][ 1 ] );
		//temp1.z = - sin( radiansS2[ i ][ 0 ] / 2 ) * cos( radiansS2[ i ][ 1 ] );
		//temp1.r = 0;
		//temp1.g = 255;
		//temp1.b = 255;
		//sampS2Hopf->points.push_back( temp1 );
	}
	sampS2Hopf->width = sampS2Hopf->points.size();
	sampS2Hopf->height = 1;


	/*** 3次元点群の VIEWER 表示 ***/
	pcl::visualization::CloudViewer viewer1( "Simple Cloud Viewer 1" );	// Viewer の名前を「 Simple Cloud Viewer 」に設定
	viewer1.showCloud( sampS2Hopf );	// 点群の表示
	while( !viewer1.wasStopped() ){}	// Viewer が閉じられるまで無限ループ 
	// [ Viewer の操作方法 ]
	// ・左ドラッグ - 視点の回転
	// ・Shift + 左ドラッグ - 視点の平行移動
	// ・Ctrl + 左ドラッグ - 画面上の回転
	// ・右ドラッグ - ズーム
	// ・g：メジャーの表示
	// ・j：スクリーンショットの保存


	/*** 3次元点群の出力保存 ***/
	
	/* 点群( PCD データ )の書き込み */
	cout << " PCD データ( PointXYZRGB )の出力保存中..." << endl;
	if( pcl::io::savePCDFileASCII( outputSamplingS2HopfPCDFileName, *sampS2Hopf ) == -1 ){ cout << " PCD データの書き込み失敗。" << endl << " " << outputSamplingS2HopfPCDFileName << " が出力保存できませんでした。" << endl; return( -1 ); }
	cout << " PCD データの出力保存 OK!" << endl;

	/* 点群( PLY データ )の書き込み */
	cout << " PLY データ( PointXYZRGB )の出力保存中..." << endl;
	if( pcl::io::savePLYFileASCII( outputSamplingS2HopfPLYFileName, *sampS2Hopf ) == -1 ){ cout << " PLY データの書き込み失敗。" << endl << " " << outputSamplingS2HopfPLYFileName << " が出力保存できませんでした。" << endl; return( -1 ); }
	cout << " PLY データの出力保存 OK!" << endl;


	/*** 球面座標S2上の3次元点群の作成 ( 通常の球面座標表現 ) ***/
	cout << "【 球面座標S2上の3次元点群の可視化 ( 通常の球面座標表現 ) 】" << endl;

	/* 3次元点群の作成 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampS2HEALPix( new pcl::PointCloud<pcl::PointXYZRGB> );
	pcl::PointXYZRGB temp2;
	for( int i = 0; i < radiansS2.size(); i++ ){
		temp2.x = sin( radiansS2[ i ][ 0 ] ) * cos( radiansS2[ i ][ 1 ] );
		temp2.y = sin( radiansS2[ i ][ 0 ] ) * sin( radiansS2[ i ][ 1 ] );
		temp2.z = cos( radiansS2[ i ][ 0 ] );
		temp2.r = 0;
		temp2.g = 255;
		temp2.b = 0;
		sampS2HEALPix->points.push_back( temp2 );
	}
	sampS2HEALPix->width = sampS2HEALPix->points.size();
	sampS2HEALPix->height = 1;


	/*** 3次元点群の VIEWER 表示 ***/
	pcl::visualization::CloudViewer viewer2( "Simple Cloud Viewer 2" );	// Viewer の名前を「 Simple Cloud Viewer 」に設定
	viewer2.showCloud( sampS2HEALPix );	// 点群の表示
	while( !viewer2.wasStopped() ){}	// Viewer が閉じられるまで無限ループ 
	// [ Viewer の操作方法 ]
	// ・左ドラッグ - 視点の回転
	// ・Shift + 左ドラッグ - 視点の平行移動
	// ・Ctrl + 左ドラッグ - 画面上の回転
	// ・右ドラッグ - ズーム
	// ・g：メジャーの表示
	// ・j：スクリーンショットの保存


	/*** 3次元点群の出力保存 ***/
	
	/* 点群( PCD データ )の書き込み */
	cout << " PCD データ( PointXYZRGB )の出力保存中..." << endl;
	if( pcl::io::savePCDFileASCII( outputSamplingS2HEALPixPCDFileName, *sampS2HEALPix ) == -1 ){ cout << " PCD データの書き込み失敗。" << endl << " " << outputSamplingS2HEALPixPCDFileName << " が出力保存できませんでした。" << endl; return( -1 ); }
	cout << " PCD データの出力保存 OK!" << endl;

	/* 点群( PLY データ )の書き込み */
	cout << " PLY データ( PointXYZRGB )の出力保存中..." << endl;
	if( pcl::io::savePLYFileASCII( outputSamplingS2HEALPixPLYFileName, *sampS2HEALPix ) == -1 ){ cout << " PLY データの書き込み失敗。" << endl << " " << outputSamplingS2HEALPixPLYFileName << " が出力保存できませんでした。" << endl; return( -1 ); }
	cout << " PLY データの出力保存 OK!" << endl;


	/* 処理終了のコール */
	cout << "【 プログラム終了 】" << endl;


	return 0;
}






#ifdef CUT
	
	ifstream file( inputS2FileName );
	vector< vector< string > > values;
	string str;
	int p;
	
	if( file.fail() ){
		cerr << "failed." << endl;
		exit( 0 );
	}
	
    while( getline( file, str ) ){
		
		/* コメント箇所は除く */
		if( ( p = str.find( "//" ) ) != str.npos ) continue;
		vector< string > inner;
		
		/* コンマがあるかを探し、そこまでをvaluesに格納 */
		while( ( p = str.find( "," ) ) != str.npos ){
			inner.push_back( str.substr( 0, p ) );
			
			//strの中身は", "の2文字を飛ばす
			str = str.substr( p + 2 );
		}
		
		inner.push_back( str );
		values.push_back( inner );
	}

    for( unsigned int i = 0; i < values.size(); ++i){
        for( unsigned int j = 0; j < values[i].size(); ++j){
            cout << values[i][j] << ",";
        }
        cout << endl;
    }

#endif