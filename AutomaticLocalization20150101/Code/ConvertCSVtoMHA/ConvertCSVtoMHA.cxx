/***** CSV データから MHA データへの変換 *****/

// 【 コマンドライン引数 】
// 1. 入力ボリュームデータ名.csv
// 2. 出力ボリュームデータ名.mha


/*** インクルードファイル ***/

/* ITK 4.5.2 */
#include "itkImage.h"							// 画像データ型
#include "itkImageFileReader.h"					// 画像の書き込み
#include "itkImageFileWriter.h"					// 画像の読み込み
#include "itkCastImageFilter.h"					// 画像のデータ型の変換
#include "itkRescaleIntensityImageFilter.h"		// ピクセル値のリスケール
#include "itkCannyEdgeDetectionImageFilter.h"	// CannyEdgeDetection フィルタ
#include "itkImageRegionIteratorWithIndex.h"	// Iterator

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/*** 名前空間の宣言 ***/
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char* argv[] ){


	/* プログラム開始のコール */
	cout << "----- CSV データから MHA データへの変換 -----" << endl;

	/* コマンドライン引数読み込み失敗時 */
	if( argc < 3 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << " inputFileName.csv outputFileName.mha" << endl;
		return EXIT_FAILURE;
	}

	/* 変数の定義 */
	const char *inputFileName = argv[ 1 ];	// 入力ボリュームデータ名( .csv )
	const char *outputFileName = argv[ 2 ];	// 出力ボリュームデータ名( .mha, .mhd, … )
	FILE *inputFile;						// ファイル型変数
	vector< vector< int > > pixels;		// ボクセルデータの座標値と画素値の集まり
	vector< int > pixel;					// ボクセルデータの座標値と画素値
	int XSize = 0;
	int YSize = 0;
	int ZSize = 0;
	int largestPixelValue = 0;
	//int temp;
	//int counter = 0;


	/*** エクセルファイルの読み込み ***/

	/* ファイル読み込み失敗時 */
	if( ( inputFile = fopen( inputFileName, "r" ) ) == NULL ){
		cerr << "ファイル " << inputFileName << " が開けません" << endl;
		return EXIT_FAILURE;
	}

	/* メモリの確保 */
	pixels.resize( 0 );	// 2次元配列の要素数
	pixel.resize( 4 );	// 1次元配列の要素数

	/* ファイル内容の読み込み */
	cout << ">>> CSV ファイルの読み込み中" << "\r";
	while( fscanf( inputFile, "%d,%d,%d,%d", &pixel[ 0 ], &pixel[ 1 ], &pixel[ 2 ], &pixel[ 3 ] ) != EOF ){

#ifdef COMMENT1
		cout << pixel[ 0 ] << " " << pixel[ 1 ] << " " << pixel[ 2 ] << " " << pixel[ 3 ] << endl;
#endif
		/* ボリュームデータサイズの算出 */
		if( pixel[ 0 ] > XSize ) XSize = pixel[ 0 ];
		if( pixel[ 1 ] > YSize ) YSize = pixel[ 1 ];
		if( pixel[ 2 ] > ZSize ) ZSize = pixel[ 2 ];
		
		/* 最大画素値の算出 */
		if( pixel[ 3 ] > largestPixelValue ) largestPixelValue = pixel[ 3 ];
		
		/* 2次元配列への1次元配列の追加 */
		pixels.push_back( pixel );	// ベクトルの末尾に第1引数を追加する
	}
	cout << ">>> CSV ファイルの読み込み OK" << endl;


#ifdef COMMENT2
	/* 読み込んだボクセルデータのコマンドライン出力 */
	cout << "【 X, Y, Z, Intensity 】" << endl;
	for( int i = 0; i < pixels.size(); i++ ){
		for( int j = 0; j < pixels[ i ].size(); j++ ){
			cout << pixels[ i ][ j ] << " ";
		}
		cout << endl;
	}
#endif


	/*** ボリュームデータの書き出し：3DSlicer で読み込めるデータ型に変換 ***/
	
	/* データ型の定義 */
	typedef int PixelType;	// 入出力画像のピクセルのデータ型：unsigned char
	const unsigned int Dimension = 3;		// 入力画像データの次元数の指定

	/* 画像型の定義 */
	typedef itk::Image< PixelType, Dimension > OutputImageType;

	/* ファイルストリーム型の定義 */
	typedef itk::ImageFileWriter< OutputImageType > WriterType;	// RealImageType 型の画像を書き出すファイルストリーム 

	/* メモリの確保 */
	OutputImageType::Pointer outputImage = OutputImageType::New();
	WriterType::Pointer writer = WriterType::New();	// 書き込み専用ファイルストリーム型変数
	
	/* ファイル名の指定 */
	writer->SetFileName( outputFileName );	// 書き込み専用ファイルストリーム型変数に出力ファイル名を指定

	/* 画像サイズ */
	const OutputImageType::SizeType size = { { XSize, YSize, ZSize } };
	const OutputImageType::IndexType start = { { 0, 0, 0 } };
	int n = 0;

	/* 画像領域の指定 */
	OutputImageType::RegionType region;
	region.SetSize( size );
	region.SetIndex( start );

	outputImage->SetRegions( region );
	outputImage->Allocate();

	OutputImageType::PixelType pixelValue;
	
	/* ボリュームデータの型変換 */
	cout << ">>> ボリュームデータの型変換中" << "\r";
	for( int z = 0; z < ZSize; z++ ){
		for( int y = 0; y < YSize; y++ ){
			for( int x = 0; x < XSize; x++ ){

				OutputImageType::IndexType pixelIndex = { { x, y, z } };
				OutputImageType::PixelType pixelValue = pixels[ n ][ 3 ];
				outputImage->SetPixel( pixelIndex, pixelValue );

				n++;
			}
		}
	}
	cout << ">>> ボリュームデータの型変換 OK" << endl;


	/*** 結果の出力 ***/
	cout << ">>> ボリュームデータ出力中" << "\r";

	/* 書き出し専用ファイルストリーム型変数にボリュームデータを設定 */
	writer->SetInput( outputImage );

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


	return 0;
}

