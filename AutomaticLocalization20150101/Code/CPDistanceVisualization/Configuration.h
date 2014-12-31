/***** 設定ヘッダファイル *****/


/*** フラグの定義 ***/
#define MULTIPLY_NUM 10	// 対応点間の距離値をこの値倍する：100で小数第二位まで残す 


/*** 定数の定義 ***/

/* 入力ファイルパス */
//const char CPDistanceFileName[] = "Output/ExhaustiveSearchResult/CPDistance.csv";		// 全探索後の対応点(最近点)間の距離
const char CPDistanceFileName[] = "Output/ICPResult/CPDistance.csv";	// ICPによる最適化後の対応点(最近点)間の距離

/* 出力ファイルパス */
//const char HistogramFileName[] = "Output/Histogram/AfterExhaustiveSearch/CPDistanceAfterES.csv";		// ヒストグラムのビンの出力先エクセルファイル名(全探索後)
const char HistogramFileName[] = "Output/Histogram/AfterICP/CPDistanceAfterICP.csv";	// ヒストグラムのビンの出力先エクセルファイル名(最適化後)
//const char HistogramImageFileName[] = "Output/Histogram/AfterExhaustiveSearch/CPDistanceAfterES.png";	// ヒストグラム画像(全探索後)
const char HistogramImageFileName[] = "Output/Histogram/AfterICP/CPDistanceAfterICP.png";	// ヒストグラム画像(最適化後)
