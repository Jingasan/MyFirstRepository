/***** MarchingCubes 法による表面抽出 *****/
// 入力となるマスクボリュームデータからモデルの表面をメッシュデータとして出力する
// メッシュデータは頂点の3次元座標群とポリゴンを構成するメッシュのID番号群で構成される

// コマンドライン引数
// 1. 入力ボリュームデータ名.mha
// 2. 出力メッシュデータ名.vtk
// 3. 物体領域部分の画素値

// 例1：
// Output/MultiResolutionalVolumeData/LiverID_0002_0.mha Output/MeshData/LiverID_0002/LiverID_0002_0.vtk 1
// Output/MultiResolutionalVolumeData/LiverID_0002_1.mha Output/MeshData/LiverID_0002/LiverID_0002_1.vtk 1
// Output/MultiResolutionalVolumeData/LiverID_0002_2.mha Output/MeshData/LiverID_0002/LiverID_0002_2.vtk 1
// Output/MultiResolutionalVolumeData/LiverID_0002_3.mha Output/MeshData/LiverID_0002/LiverID_0002_3.vtk 1


/*** インクルード ***/

/* C++ */
#include <iostream>

/* ITK 4.5.2 */
#include "itkImageFileReader.h"
#include "itkBinaryMask3DMeshSource.h"
#include "itkImage.h"
#include "itkObject.h"
#include "itkVTKPolyDataWriter.h"

/* 設定ヘッダファイル */
#include "Configuration.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char * argv[] ){
	
	
	/* コマンドライン引数読み込み失敗時 */
	if( argc < 3 ){
		cerr << "Usage: IsoSurfaceExtraction inputImageFile outputMeshFile objectValue " << endl;
		return EXIT_FAILURE;
	}


	/* プログラム開始のコール */
	cout << "【 プログラム開始 】" << endl;


	/*** 各種データ型、クラス型の定義 ***/

	/* データ型の定義 */
	const unsigned int Dimension = 3;						// 入力画像データの次元数の指定
	typedef unsigned char PixelType;						// 入力画像のピクセルのデータ型：unsigned char

	/* 画像型、メッシュ型の定義 */
	typedef itk::Image< PixelType, Dimension > ImageType;	// 画像型：unsigned char, 次元数：3
	typedef itk::Mesh< double > MeshType;					// メッシュ型：double

	/* ファイルストリーム型の定義 */
	typedef itk::ImageFileReader< ImageType > ReaderType;	// ImageType 型の画像を読み込むファイルストリーム
	typedef itk::VTKPolyDataWriter< MeshType > WriterType;	// MeshType 型のメッシュを書き込むファイルストリーム
	
	/* 変数の定義とメモリの確保 */
	ReaderType::Pointer reader = ReaderType::New();	// 読み込み専用ファイルストリーム型変数
	WriterType::Pointer writer = WriterType::New();	// 書き込み専用ファイルストリーム型変数

	/* ファイル名の指定 */
	reader->SetFileName( argv[ 1 ] );	// 読み込み専用ファイルストリーム型変数に入力ファイル名を指定
	writer->SetFileName( argv[ 2 ] );


	/*** メッシュ化対象のボリュームデータの読み込み ***/

	/* ファイルの読み込み */
	try{
		reader->Update();
	}
	catch( itk::ExceptionObject & exp ){
		cerr << "Exception thrown while reading the input file " << endl;
		cerr << exp << endl;
		return EXIT_FAILURE;
	}
	

	/*** 入力データ情報の取得 ***/

	/* 入力画像の取得 */
	ImageType::Pointer image = reader->GetOutput();
	
	/* 入力ボリュームデータのプロパティのコマンドライン出力 */
	cout << endl << ">> 入力データ情報" << endl;
	cout << "1voxelの X 方向サイズ：" << image->GetSpacing().GetElement( 0 ) << endl;
	cout << "1voxelの Y 方向サイズ：" << image->GetSpacing().GetElement( 1 ) << endl;
	cout << "1voxelの Z 方向サイズ：" << image->GetSpacing().GetElement( 2 ) << endl;
	cout << "ボリュームデータの原点 X 座標：" << image->GetOrigin().GetElement( 0 ) << endl;
	cout << "ボリュームデータの原点 Y 座標：" << image->GetOrigin().GetElement( 1 ) << endl;
	cout << "ボリュームデータの原点 Z 座標：" << image->GetOrigin().GetElement( 2 ) << endl;
	cout << "ボリュームデータの X 方向サイズ：" << image->GetLargestPossibleRegion().GetSize( 0 ) << endl;
	cout << "ボリュームデータの Y 方向サイズ：" << image->GetLargestPossibleRegion().GetSize( 1 ) << endl;
	cout << "ボリュームデータの Z 方向サイズ：" << image->GetLargestPossibleRegion().GetSize( 2 ) << endl << endl;


	/*** メッシュ化 ***/
	cout << "【 MarchingCubes 法 によるメッシュ化 】" << endl;

	/* BinaryMask3DMeshSource クラスの定義 */
	typedef itk::BinaryMask3DMeshSource< ImageType, MeshType > MeshSourceType;	// ボリュームデータのメッシュ化

	/* メモリの確保 */
	MeshSourceType::Pointer meshSource = MeshSourceType::New();
	
	/* マスクボリュームデータにおける物体領域部分の輝度値の取得 */
	const PixelType objectValue = static_cast< PixelType >( atof( argv[ 3 ] ) );
	
	/* 表面抽出フィルタ */
	meshSource->SetObjectValue( objectValue );		// マスクボリュームデータの物体領域部分の輝度値を指定
	meshSource->SetInput( reader->GetOutput() );	// 入力したマスクボリュームデータを指定

	/* マスクボリュームデータの MarchingCubes 法を用いたメッシュ化 */
	cout << ">> メッシュ生成中" << "\r";
	try{
		meshSource->Update();
	}
	catch( itk::ExceptionObject & exp ){
		cerr << "Exception thrown during Update() " << endl;
		cerr << exp << endl;
		return EXIT_FAILURE;
	}
	cout << ">> メッシュ生成完了" << endl;


	/*** 出力データ情報の取得 ***/
	cout << endl;
	cout << ">> 出力データ情報の出力" << endl;
	cout << "メッシュ頂点数：" << meshSource->GetNumberOfNodes() << endl;	// 頂点数( Nodes )
	cout << "ポリゴン数：" << meshSource->GetNumberOfCells() << endl;		// ポリゴン数( Cells )
	cout << endl;

	
	/*** メッシュデータの出力 ***/
	cout << ">> メッシュ出力中" << "\r";

	/* MarchingCubes 法により得られたメッシュを出力に指定 */
	writer->SetInput( meshSource->GetOutput() );
	
	/* メッシュの出力 */
	writer->Write();


	/* プログラム終了のコール */
	cout << ">> メッシュ出力完了" << endl;
	cout << "【 プログラム終了 】" << endl;


	return EXIT_SUCCESS;
}
