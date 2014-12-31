/***** 3次元 Canny エッジ検出 *****/

// 【 コマンドライン引数 】
// 1. 入力ボリュームデータ名.拡張子 
// 2. 出力ボリュームデータ名.拡張子
// 3. 出力点群データ名.ply
// 4. 出力点群データ名.pcd
// 5. 分散値：ガウシアンフィルタの窓サイズ
// 6. エッジ連結の上限閾値
// 7. エッジ連結の下限閾値
// 8. ボクセルグリッドサイズ

// 例 1：
// Input/VolumeData/ID_0002.mhd Output/EdgeVolumeData/ID_0002_LiverV31U10L1.mha Output/EdgePointData/ID_0002_LiverV31U10L1.ply Output/EdgePointData/ID_0002_LiverV31U10L1.pcd 31 10 1

// 例 2：
// Input/VolumeData/LiverID_0002.mhd Output/EdgeVolumeData/LiverID_0002V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002V5U0.01L0.01.pcd 5 0.01 0.01

// 例 3：
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_0.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_0V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_0V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_0V5U0.01L0.01.pcd 5 0.01 0.01
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_1.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_1V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_1V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_1V5U0.01L0.01.pcd 5 0.01 0.01
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_2.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_2V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_2V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_2V5U0.01L0.01.pcd 5 0.01 0.01
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_3.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_3V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_3V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_3V5U0.01L0.01.pcd 5 0.01 0.01

// 例 4：
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_0.mha Output/EdgeVolumeData/ID_0002/ID_0002_0V31U10L1.mha Output/EdgePointData/ID_0002/ID_0002_0V31U10L1.ply Output/EdgePointData/ID_0002/ID_0002_0V31U10L1.pcd 31 10 1
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_1.mha Output/EdgeVolumeData/ID_0002/ID_0002_1V16U10L1.mha Output/EdgePointData/ID_0002/ID_0002_1V16U10L1.ply Output/EdgePointData/ID_0002/ID_0002_1V16U10L1.pcd 16 10 1
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_2.mha Output/EdgeVolumeData/ID_0002/ID_0002_2V9U10L1.mha Output/EdgePointData/ID_0002/ID_0002_2V9U10L1.ply Output/EdgePointData/ID_0002/ID_0002_2V9U10L1.pcd 9 10 1
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_3.mha Output/EdgeVolumeData/ID_0002/ID_0002_3V5U10L1.mha Output/EdgePointData/ID_0002/ID_0002_3V5U10L1.ply Output/EdgePointData/ID_0002/ID_0002_3V5U10L1.pcd 5 10 1


//  This example introduces the use of the CannyEdgeDetectionImageFilter.
//  This filter is widely used for edge detection
//  since it is the optimal solution satisfying the constraints of good sensitivity, localization and noise robustness.


/*** インクルードファイル ***/

/* ITK 4.5.2 */
#include "itkImage.h"							// 画像データ型
#include "itkImageFileReader.h"					// 画像の書き込み
#include "itkImageFileWriter.h"					// 画像の読み込み
#include "itkCastImageFilter.h"					// 画像のデータ型の変換
#include "itkRescaleIntensityImageFilter.h"		// ピクセル値のリスケール
#include "itkCannyEdgeDetectionImageFilter.h"	// CannyEdgeDetection フィルタ
#include "itkImageRegionIteratorWithIndex.h"	// Iterator

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// 点群のデータ型

/* C */
#include <windows.h>	// 処理時間計測

/* 作成ヘッダファイル */
#include "Configuration.h"		// 設定ヘッダファイル
#include "FunctionDefinition.h"	// 関数の定義


/*** 名前空間の宣言 ***/
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char* argv[] ){


	/* プログラム開始のコール */
	DWORD wholeTimeStart = GetTickCount();	// 処理全体の時間計測開始
	cout << "【 プログラム開始 】" << endl;

	/* コマンドライン引数読み込み失敗時 */
	if( argc < 5 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl;
		cerr << "1. InputImage" << endl;
		cerr << "2. OutputImage" << endl;
		cerr << "3. OutputPlyFile" << endl;
		cerr << "4. OutputPCDFile" << endl;
		cerr << "[ 5. variance ]" << endl;
		cerr << "[ 6. upperThreshold ]" << endl;
		cerr << "[ 7. lowerThreshold ]" << endl;
		cerr << "[ 8. voxelGridSize ]" << endl;
		system( "pause" );
		return EXIT_FAILURE;
	}



	/* 変数の宣言 */
	const char * inputFilename  = argv[ 1 ];	// 入力ボリュームデータ名
	const char * outputFilename1 = argv[ 2 ];	// 出力ボリュームデータ名
	const char * outputFilename2 = argv[ 3 ];	// 出力点群データ名
	const char * outputFilename3 = argv[ 4 ];	// 出力点群データ名
	
	/* 各種プロパティの初期値設定 */
	float variance = VARIANCE;
	float upperThreshold = UPPERTHRESHOLD;
	float lowerThreshold = LOWERTHRESHOLD;
	float voxelGridSizeX = VOXELGRIDSIZE;
	float voxelGridSizeY = VOXELGRIDSIZE;
	float voxelGridSizeZ = VOXELGRIDSIZE;

	/* CannyEdgeDetection フィルタに対する各種値の読み込み */
	if( argc > 5 ){ variance = atof( argv[ 5 ] ); }			// 分散の値
	if( argc > 6 ){ upperThreshold = atof( argv[ 6 ] ); }	// Canny フィルタの上限閾値
	if( argc > 7 ){ lowerThreshold = atof( argv[ 7 ] ); }	// Canny フィルタの下限閾値
	if( argc > 8 ){
		voxelGridSizeX = atof( argv[ 8 ] );	// ボクセルグリッド X サイズ[単位:m]
		voxelGridSizeY = atof( argv[ 8 ] );	// ボクセルグリッド Y サイズ[単位:m]
		voxelGridSizeZ = atof( argv[ 8 ] );	// ボクセルグリッド Z サイズ[単位:m]
	}

	/* 入力プロパティ値のコマンドライン出力 */
	cout << "Variance = " << variance << endl;
	cout << "UpperThreshold = " << upperThreshold << endl;
	cout << "LowerThreshold = " << lowerThreshold << endl;
	

	/*** 各種データ型、クラス型の定義 ***/

	/* データ型の定義 */
	typedef unsigned char CharPixelType;	// 入出力画像のピクセルのデータ型：unsigned char
	typedef double RealPixelType;			// CannyFilter の出力結果画像のピクセルのデータ型：double
	const unsigned int Dimension = 3;		// 入力画像データの次元数の指定

	/* 画像の型の定義 */
	typedef itk::Image< CharPixelType, Dimension > CharImageType;	// ピクセル型：unsigned char, 次元数：3
	typedef itk::Image< RealPixelType, Dimension > RealImageType;	// ピクセル型：double, 次元数：3

	/* ファイルストリーム型の定義 */
	typedef itk::ImageFileReader< CharImageType > ReaderType;	// CharImageType 型の画像を読み込むファイルストリーム
	typedef itk::ImageFileWriter< CharImageType > WriterType;	// RealImageType 型の画像を書き出すファイルストリーム 
	
	/* 入力画像の画素値のデータ型( unsigned char )をフィルタリング処理で扱うデータ型( double )に変換するためのクラスの定義 */
	typedef itk::CastImageFilter< CharImageType, RealImageType > CastToRealFilterType;
	// This filter operates on image of pixel type float.
	// It is then necessary to cast the type of the input images that are usually of integer type.
	// The CastImageFilter is used here for that purpose.
	// Its image template parameters are defined for casting from the input type to the float type using for processing.
	
	/* 画素値のスケーリング変換用クラスの定義 */
	typedef itk::RescaleIntensityImageFilter< RealImageType, CharImageType > RescaleFilter;	// 二値：0-1をグレースケール値：0-255に変換するためのクラス

	/* CannyEdgeDetectionImageFilter クラスの定義 */
	typedef itk::CannyEdgeDetectionImageFilter< RealImageType, RealImageType > CannyFilter;
	// CannyEdgeDetectionImageFilter は小数値を使用する
	// The CannyEdgeDetectionImageFilter is instantiated using the float image type.


	/*** オブジェクト変数の宣言 ***/

	/* メモリの確保 */
	ReaderType::Pointer reader = ReaderType::New();						// 読み込み専用ファイルストリーム型変数
	WriterType::Pointer writer = WriterType::New();						// 書き込み専用ファイルストリーム型変数
	CastToRealFilterType::Pointer toReal = CastToRealFilterType::New();	// データ型の変換用変数
	RescaleFilter::Pointer rescale = RescaleFilter::New();				// 画素値0-1 ⇒ 0-255のスケーリング変換用変数
	CannyFilter::Pointer cannyFilter = CannyFilter::New();				// CannyEdgeDetectionImageFilter クラスの変数
	

	/*** ボリュームデータの入力 ***/

	/* ファイル名の指定 */
	cout << ">>> ボリュームデータ入力中" << "\r";
	reader->SetFileName( inputFilename );	// 読み込み専用ファイルストリーム型変数に入力ファイル名を指定
	cout << ">>> ボリュームデータ入力 OK" << endl;
	//try{
		//reader->Update();
	//}
	//catch( itk::ExceptionObject & exp ){
		//cerr << "Exception thrown while reading the input file " << endl;
		//cerr << exp << endl;
		//return EXIT_FAILURE;
	//}


	/*** 画素値のスケーリング変換の設定 ***/
	rescale->SetOutputMinimum( 0 );				// 下限：0
	rescale->SetOutputMaximum( 255 );			// 上限：255
	// The output of an edge filter is 0 or 1

	/*** 画素値のデータ型の変換 ***/
	toReal->SetInput( reader->GetOutput() );	// 入力画像ファイルを指定



	/*** 3次元エッジの抽出 ***/

	/* ボリュームデータのエッジ検出開始のコール */
	cout << "----- Canny エッジ検出開始 -----" << endl;
	DWORD cannyTimeStart = GetTickCount();	// Canny エッジ抽出の時間計測開始

	/* CannyEdgeDetectionImageFilter に対するプロパティの設定 */
	cannyFilter->SetInput( toReal->GetOutput() );		// CannyEdgeDetectionImageFilter に対する入力データ
	cannyFilter->SetVariance( variance );				// Canny フィルタの大きさ
	cannyFilter->SetUpperThreshold( upperThreshold );	// Canny フィルタの上限閾値
	cannyFilter->SetLowerThreshold( lowerThreshold );	// Canny フィルタの下限閾値
	
	/* 画素値のスケーリング変換 */
	rescale->SetInput( cannyFilter->GetOutput() );	// CannyFilter の出力エッジ画像の値が 0-1 なので、0-255 に変換し直す
	
	
	/*** ボリュームデータの出力 ***/
	cout << ">>> ボリュームデータ出力中" << "\r";

	/* 出力ファイル名の指定 */
	writer->SetFileName( outputFilename1 );	// 書き込み専用ファイルストリーム型変数に出力ファイル名を指定

	/* 書き出し専用ファイルストリーム型変数にリスケールされたエッジ抽出画像を設定 */
	writer->SetInput( rescale->GetOutput() );
	
	/* 処理結果となる3次元エッジボクセルデータの出力 */
	try{
		writer->Update();
	}

	/* 出力失敗時 */
	catch( itk::ExceptionObject & err ){
		cout << "ExceptionObject caught !" << endl;
		cout << err << endl;
		return EXIT_FAILURE;
	}
	cout << ">>> ボリュームデータ出力 OK" << endl;

	/* ボリュームデータのエッジ検出終了のコール */
	DWORD cannyTimeEnd = GetTickCount();	// Canny エッジ抽出の時間計測終了
	cout << "----- Canny エッジ検出終了 -----" << endl;


	/*** 3次元点群データの作成 ***/

	/* 3次元点群データ作成開始のコール */
	cout << "----- 3次元エッジ点群データの作成開始 -----" << endl;
	DWORD generateEdgePointCloudTimeStart = GetTickCount();

	/* エッジ抽出の結果画像の取得 */
	CharImageType::Pointer edgeImage = rescale->GetOutput();

	/* XYZ 方向の画像サイズの取得 */
	CharImageType::RegionType edgeImageRegion = edgeImage->GetLargestPossibleRegion();
	
	/* エッジ抽出画像の次元数、開始通し番号、サイズの出力 */
	cout << edgeImageRegion << endl;

	/* 繰り返し処理用クラス ImageRegionIteratorWithIndex の変数の宣言 */
	itk::ImageRegionIteratorWithIndex< CharImageType > imageIterator( edgeImage, edgeImageRegion );	// 座標値と画素値の両方を操作する際に使用

	/* PointCloud 型変数の宣言とメモリの確保 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr eCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// エッジ上の3次元点群
	pcl::PointXYZRGB tmp;

	/* Iterator の初期化 */
	imageIterator.Begin();

	/* 指定した画像領域内をループ */
	while( !imageIterator.IsAtEnd() ){

		/* エッジ点の座標値のみを取得 … エッジ上のピクセル値：255, それ以外：0 */
		if( (int)imageIterator.Get() > 127 ){

			tmp.x = imageIterator.GetIndex().GetElement( 0 );	// x 座標値
			tmp.y = imageIterator.GetIndex().GetElement( 1 );	// y 座標値
			tmp.z = imageIterator.GetIndex().GetElement( 2 );	// z 座標値
			tmp.r = 255; tmp.g = 0; tmp.b = 255;						// 点群の色：桃色に設定
			eCloud->points.push_back( tmp );							// 点群を PUSH で格納していく
			// ⇒ 座標値 ( x, y, z ) の配列が複数積まれた2次元配列のイメージ
		}

		/* ループの更新 */
		++imageIterator;
	}

	/* エッジ点群がなかった場合 */
	if( eCloud->points.size() == 0 ){ cout << "Caution!: エッジ点群なし" << endl; return 0; }

	/* 点群データ数の取得とメモリ数の設定 */
	eCloud->width = eCloud->points.size();
	eCloud->height = 1;

	/* エッジ点群数の出力 */
	DWORD generateEdgePointCloudTimeEnd = GetTickCount();
	cout << ">>> エッジ点群数: " << eCloud->points.size() << " 点" << endl;

	/* 3次元点群データ作成終了のコール */
	cout << "----- 3次元エッジ点群データの作成終了 -----" << endl;



	/* プログラム終了のコール */
	cout << "【 プログラム終了 】" << endl;
	DWORD wholeTimeEnd = GetTickCount();	// 処理全体の時間計測終了
	
	/* 各種処理時間のコマンドライン出力 */
	cout << endl << "【 ResultTime 】" << endl;
	cout << "CannyEdgeDetectionTime：" << (double)( cannyTimeEnd - cannyTimeStart ) / 1000 << " sec." << endl;
	cout << "EdgePointCloudGenerationTime：" << (double)( generateEdgePointCloudTimeEnd - generateEdgePointCloudTimeStart ) / 1000 << " sec." << endl;
	cout << "WholeTime：" << (double)( wholeTimeEnd - wholeTimeStart ) / 1000 << " sec." << endl;


	/* 3次元点群データの表示 */
	cout << "----- 3次元エッジ点群データの表示 -----" << endl;
	cout << ">>> 表示中" << "\r";
	showPointCloudRGB( eCloud );
	cout << ">>> 表示終了" << endl;


#ifdef REDUCTION

	/* 3次元点群数の削減 */
	cout << " 点群数削減中..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr reCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	reductPointCloudRGB( eCloud, reCloud, voxelGridSizeX, voxelGridSizeY, voxelGridSizeZ );
	cout << " 点群数削減完了" << endl;
	cout << " >>> BeforeCloudSize： " << eCloud->size() << " [points]" << endl;
	cout << " >>> AfterCloudSize： " << reCloud->size() << " [points]" << endl;

	/* 3次元点群データの表示 */
	cout << "----- 3次元エッジ点群データの表示 -----" << endl;
	showPointCloudRGB( reCloud );
	
	/* 3次元点群データの出力保存 */
	cout << "----- 3次元エッジ点群データの出力保存 -----" << endl;
	savePointCloudRGBtoPLY( reCloud, outputFilename2 );
	savePointCloudRGBtoPCD( reCloud, outputFilename3 );
	
#else

	/* 3次元点群データの出力保存 */
	cout << "----- 3次元エッジ点群データの出力保存 -----" << endl;
	savePointCloudRGBtoPLY( eCloud, outputFilename2 );
	savePointCloudRGBtoPCD( eCloud, outputFilename3 );

#endif

	return EXIT_SUCCESS;
}


