/*
 * mtxCV.h
 *
 *  Created on: Apr 15, 2024
 *      Author: AJ
 */

#ifndef INC_MTXCV_H_
#define INC_MTXCV_H_
//#define _mtxCV_C_BUILD_FLAG //for non .cpp file to flag c compiler to correctly link c++ functions

#ifdef _mtxCV_C_BUILD_FLAG
    #ifdef __cplusplus
        extern "C" {
    #endif
#endif

    #define _USE_MATH_DEFINES //enables use of M_PI etc
    #include <math.h>
    #include <vector> //dynamic mem
	#include <queue> //dynamic mem
	#include <unordered_map> //dynamic mem
	#include <random> //random num gen
	#include <numeric>  // for std::iota
    #include <optional> // std lib: optional

    #include "Eigen/Dense" //Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header files
    using namespace std; //< Standard lib: vector, cout etc...
    using namespace Eigen; //< Eigen vector and matrix manipulation

	//#define _DEBUG_MTXCV //DEBUG FLAG, uncomment to enable module debugging

    #define NULL_TERMINATOR 1
	
	//ASCII Gradients
    // other alternatives: "@MBHENR#KWXDFPQASUZbdehx*8Gm&04LOVYkpq5Tagns69owz$CIu23Jcfry%1v7l+it[] {}?j|()=~!-/<>\"^_';,:`. "
    // other alternatives: "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\|()1{}[]?-_+~<>i!lI;:,"^`'. "

    //returned function enum indicating state as prime number (future implementation: multiply FnStates > 0 results unique reference)
    // entries > 0 are intended to increase in criticality, currently highest FnState encountered is returned
    enum class FnState : int16_t {
        INITIALISED = -1,
        OK = 0,
        PROCESSING_WARN = 2,
        PROCESSING_ERR = 3 // future 5, 7, 11 max to fit int16_t
        //,MULTIPLE_FAULTS = INT16_MAX
    };
	
    // FnState helper function used to interpret combination of two states (ie when a function tracks multiple calls to sub functions each returning an FnState)
    inline FnState evalFnStates(const FnState& _fnState1, const FnState& _fnState2) {
        // if both states are <= 0 (ie OK or INITIALISED) return OK, otherwise return highest of the two states
        return (_fnState1 <= FnState::OK && _fnState2 <= FnState::OK ? FnState::OK : std::max(_fnState1, _fnState2));
    }

    template<int kernelN> //setup MtxKernelfloat definition so functions can accept different fixed sixed kernels (fixed size equivelant to using dynamic MatrixXf)
    using MtxKernelfloat = Matrix<float, kernelN, kernelN>; // ### PRIMARY float KERNEL FIXED-SIZE MATRIX DEFINE ###

    typedef Array<uint8_t, 256, 1> Array256byte; // ### PRIMARY uint8_t FIXED-SIZE HISTOGRAM ARRAY DEFINE ###

//    typedef pair<Eigen::Index, Eigen::Index> Coord; // pair def coordinate as <x,y>
//    typedef pair<float, float> CoordAdjustment; // pair def coordinate decimal adjustement as +-<x,y>

    // Stores integer image coordinates of identified edge, and decimal sub-pixel correction adjustment
    struct EdgePoint {
    	Vector2i coord; // (x,y) image coords of edge
        Vector2f adjustment; // sub-pixel adjustement of (x,y)
        // Struct Constructor
        EdgePoint(int _x, int _y) : coord({_x, _y}), adjustment(0.0f, 0.0f) {}
    };

//    // A set of edge point sequences making connected contours in image
//    struct Contours {
//        vector<vector<EdgePoint*>> linkedEdges; // Vector of vectors of edge point reference pointers making a connected sequence in the image
//    };


    /////// CLASS DECLERATION ///////
	class MtxCV {
	private:
		//treasure_map//
	public:
		/*Function to compute !factorial via loop
		Parameters:
			- n: Number to calculate factorial for
		Returns: Factorial sum of n */
		static inline int factorial(const int& n) { //TBD check variable size appropriate to application
			int result = 1;
			for (int i = 2; i <= n; ++i) {
				result *= i;
			}
			return result;
		}
		/* Variant of factorial() above using static memory to store values as they are calculated to speed up repeat calls
		Parameters:
			- n: Number to calculate factorial for
		Returns: Factorial sum of n */
		static inline int factorial_memo(const int& n) {
			static std::unordered_map<int, int> preCalc; //static dictionary to store factorial results as they are calculated
			// Search dict to see if calc has already been performed
			if (preCalc.find(n) != preCalc.end()) {
				return preCalc[n]; //return previously calc value
			}
			// requested factorial not calculated yet, calculate and store result
			return preCalc[n] = ((n <= 1) ? 1 : n * factorial_memo(n - 1)); //check base case 0!, 1! = 1; otherwise perform recursive calc using: n! = n×(n−1)!
		}

		/* Round float to specific decimal places using base10 offset
		Parameters:
		- inbound: float to be rounded
		- decimalPrecision: number of decimal places to round to
		Returns: Rounded float value */
		static inline float roundPrecision(const float& inbound, const uint8_t decimalPrecision) {
			float base10 = std::pow(10.0f, static_cast<float>(decimalPrecision));
			return std::round(inbound * base10) / base10;
		}

		/*Linear interpolation function takes two values val1 and val2 and a weight, typically ranging from 0 to 1, and performs linear interpolation between them.
		Parameters:
		- val1: Value to interpolate from (100% - weight of this value)
		- val2: Value to interpolate to (weight is % of this value)
		- weight: The decimal percentage weight of val2 to val1
		Returns: Interpolated value which lies between val1 and val2 based on the provided weight */
		static inline float interpolateLinear(const float &val1, const float &val2, const float &weight) {
			// equation adapted from http://dx.doi.org/10.58286/27751
			return val1 * (1.0f - weight) + val2 * weight;
		}

		/*Function pads image with extended borders for convolution of edge pixels; corner pixels are extended in 90° wedges. Other edge pixels are extended in lines away from image.
		Parameters:
		- image: Input image MatrixXf or MatrixXi matrix to have padding applied
		- borderSize: px border size to be applied to image
		Returns: Padded version of input image */
		template<typename MatrixType>
		static MatrixXf padImageExtended(const MatrixType& image, const Index& borderSize) {
			const Index imageRows = image.rows(); //static_cast<int>()
			const Index imageCols = image.cols();
			const Index paddedRows = imageRows + (2 * borderSize);
			const Index paddedCols = imageCols + (2 * borderSize);

			// Init paddedImage with zeros to new image size with borders
			MatrixXf paddedImage(paddedRows, paddedCols);
			paddedImage.setZero();

			// Copy original image to the center of padded image (image is centre of matrix, with borderSize sized border around it)
			paddedImage.block(borderSize, borderSize, imageRows, imageCols) = image.template cast<float>();
				/*  eg assuming 3x2 image & borderSize = 1, paddedImage is 5x4 as below:
					[00][00][00][00][00]
					[00][I1][I2][I3][00]
					[00][I4][I5][I6][00]
					[00][00][00][00][00]*/

			// Extend borders by replicating the nearest border pixels
			for (int i = 0; i < borderSize; ++i) { //for each pixel the border is extended
				/*Top and Bottom borders:
					paddedImage border above and below pasted image is filled with corresponding values from image top & bottom row respectively
					[00][I1][I2][I3][00]
					[00][I1][I2][I3][00]
					[00][I4][I5][I6][00]
					[00][I4][I5][I6][00]*/
				paddedImage.row(i) = paddedImage.row(borderSize); //extend top border
				paddedImage.row(paddedRows - 1 - i) = paddedImage.row(paddedRows - 1 - borderSize); //extend bottom border

				/*Left and Right borders:
					paddedImage border left and right of pasted image is filled with corresponding values from closest left & right col respectively
					[I1][I1][I2][I3][I3]
					[I1][I1][I2][I3][I3]
					[I4][I4][I5][I6][I6]
					[I4][I4][I5][I6][I6]*/
				paddedImage.col(i) = paddedImage.col(borderSize); //extend left border
				paddedImage.col(paddedCols - 1 - i) = paddedImage.col(paddedCols - 1 - borderSize); //extend right border
			}

			return paddedImage;
		}

		/*Function prints matrix to serial due limited std::cout functionality
		Parameters:
		- inboundData: Input Eigen Matrix or Array to print to serial
		Returns: NA */
		template<typename Derived> // allows both Matrix and Array objects to be printed with simgle function
		static void printMatrix(const DenseBase<Derived> &inboundData){
			const Index rows = inboundData.rows();
			const Index cols = inboundData.cols();
			for(Index yRow = 0; yRow < rows; yRow++){
				for(Index xCol = 0; xCol < cols; xCol++){
					printf("%*.2f ", 8, static_cast<float>(inboundData(yRow, xCol)));
				}
				printf("\n\r");
			}
		}

		/*Generate Eigen matrix from 1D float array
		Parameters:
		- thermalArr: Input float array to be converted to matrix form
		- arrSize: Size of the incoming float array (eg (float)sensor.Arr)
		- matrixReturn: Return matrix, also determines rows and cols of mapping
		- minIntensityRatioCutoff: valid range 0-1, after normalisation values below this percentage of the the input intensity range are set to 0 in the output
		Returns: executed function state */
		template<typename MatrixType>
		static FnState getByteMatrixFromFloatArray(const float* thermalArr, const size_t& arrSize, MatrixType& matrixReturn) { //, MatrixXf& returnNorm
			const Index xCols = matrixReturn.cols();
			const Index yRows = matrixReturn.rows();
			//std::cout << "AT FUNCT getByteMatrixFromFloatArray: matrix:" << yRows * xCols << " array:" << arrSize << "\n\r";
			// Validate size of matrix vs array size
			if (static_cast<size_t>(yRows * xCols) != arrSize) {
				#ifdef _DEBUG_MTXCV
					//std::cerr << "[Err] @getEigenMatrixFromThermalArray: Size mismatch between provided array and matrix rows*cols.\n\r";
					printf("[Err] @getEigenMatrixFromThermalArray: Size mismatch between provided array and matrix rows*cols.\n\r");
				#endif
				return FnState::PROCESSING_ERR; // Return an empty matrix
			}

			// Create an Eigen Map to treat the input array as Matrixf so we can use Eigen functions; stride affects element to element and row to row step distance respectively
            Eigen::Map<const MatrixXf, Eigen::Unaligned, Stride<Dynamic,Dynamic>> mappedArr(thermalArr, yRows, xCols, Stride<Dynamic,Dynamic>(1, xCols));

			// init normalisation Matrix with the provided dimensions
			MatrixXf normalisedMatrix(yRows, xCols);
			// normalise input matrix from current range to 0->1 (could also do the 255.0f scaling there, but seperated for now for debugging)
			FnState fnState = normaliseMatrix(mappedArr, normalisedMatrix, 1.f);

			// after normalisation input image range is mapped 0->1, suppress intensity of pixels to 0 when they are below the minIntensityRatioCutoff
			//normalisedMatrix = (normalisedMatrix.array() < minIntensityRatioCutoff).select(0.0f, normalisedMatrix);

			// scale normalised matrix to byte range
			matrixReturn = (normalisedMatrix.array() * 255.f).template cast<typename MatrixType::Scalar>();

			// TEMP REMOVED functionality to return norm matrix to save mem space
			//returnNorm = normalisedMatrix;

			// return function execution state
			return evalFnStates(fnState, FnState::OK);
		}

//		template<typename MatrixType>
//		static FnState suppressIntensityBelowRatio(MatrixType& image, const float minIntensityRatioCutoff = 0.0f) {
//			// Find min and max values in the image
//			const float minVal = static_cast<float>(image.minCoeff());
//			const float maxVal = static_cast<float>(image.maxCoeff());
//			float cutoff = (maxVal - minVal) * minIntensityRatioCutoff;
//			// after normalisation input image range is mapped 0->1, suppress intensity of pixels to 0 when they are below the minIntensityRatioCutoff
//			image = (image.template cast<float>().array() - minVal < cutoff).select(0.0f, image);
//			// return function execution state
//			return FnState::OK;//evalFnStates(fnState, FnState::OK);
//		}

		/*Function linearly map an input float matrix from its min and max values to 0-1, this is then multiplied by the scalingfactor (eg uint8_t range (0-255))
		Parameters:
		- image: Input image to scale (eg float)
		- imgReturn: returned scaled version of image, or zero matrix on error; ensure matrix range can fit scalingFactor before calling
		- scalingFactor: scales normalised image range from 0->1 to 0->scalingFactor; ensure matrix range can fit scalingFactor before calling
		Returns: executed function state */
		template<typename MatrixTypeInput, typename MatrixTypeReturn>
		static FnState normaliseMatrix(const MatrixTypeInput& image, MatrixTypeReturn& imgReturn, const float scalingFactor = 1.0f) {
			//Validate passed matrix dimensions
			bool rowCheck = (image.rows() == imgReturn.rows());
			bool colCheck = (image.cols() == imgReturn.cols());
			// Ensure both magnitude and angle gradient matrix are same size
			if (!(rowCheck && colCheck)) {
				#ifdef _DEBUG_MTXCV
					printf("[Err] @normaliseMatrix: Input and return matrix must have the same dimensions.\n\r");
				#endif
				return FnState::PROCESSING_ERR; // return error
			}
            imgReturn.setZero(); // init returned matrix to 0

			// Find min and max values in the image
			const float minVal = static_cast<float>(image.minCoeff());
			const float maxVal = static_cast<float>(image.maxCoeff());
			// Check for potential divide by zero
			if (minVal == maxVal) {
				#ifdef _DEBUG_MTXCV
					printf("[Warn] @normaliseMatrix: Image max and min intensity are equal, a zero matrix was returned.\n\r");
				#endif
				return FnState::PROCESSING_WARN;
			}

			// Normalise image float values to 0%->100% range based on initial distribution and scale to uint8_t 0->255 range
			// imgReturn = (((image.array() - minVal) / (maxVal - minVal)) * 255.0f).template cast<typename MatrixTypeReturn::Scalar>();
			imgReturn = (((image.array() - minVal) / (maxVal - minVal)) * scalingFactor).template cast<typename MatrixTypeReturn::Scalar>();

			#ifdef _DEBUG_MTXCV
				printf("@normaliseMatrix: inbound pixels minVal:[%.2f] maxVal:[%.2f] ->mapped-> minVal:[%.2f] maxVal:[%.2f].\n\r", minVal, maxVal, imgReturn.minCoeff(), imgReturn.maxCoeff());
			#endif
			//printMatrixTableFormat(image,"normaliseMatrix.inbound\0");
			//printMatrixTableFormat(imgReturn,"normaliseMatrix.scaled\0", 2);

			return FnState::OK;
		}

//		/*Function to construct a matrix from EdgePoint vector
//		Parameters:
//		- edgePoints: vector of edge coordinates to render to matrix image
//		- imgReturn: reference to returned matrix which will be written to
//		- clearBeforeWrite: sets whether imgReturn is cleared to 0 before writing points, otherwise points are writen over existing matrix data
//		- imageReference: if imageReference is provided, the points written to imgReturn are the intensities of the corresponding location in imageReference
//		Returns: executed function state */
//		template<typename MatrixType>
//		static FnState constructMatrixFromPoints(const std::vector<EdgePoint*>& edgePoints, MatrixType& imgReturn, bool clearBeforeWrite = true, const MatrixType *imageReference = nullptr) {
//		//static FnState constructMatrixFromPoints(const std::vector<EdgePoint>& edgePoints, MtxImgbyte& imgReturn, const MtxImgbyte *imageReference = nullptr) {
//			FnState fnState = FnState::INITIALISED;
//			const uint8_t pixelValue = UINT8_MAX;
//			const Index rows = imgReturn.rows();
//			const Index cols = imgReturn.cols();
//			bool useReference = false; //flag if a reference image is provided, edge points are coloured to corresponding reference px
//			if(imageReference != nullptr && imageReference->rows() == rows && imageReference->cols() == cols) { //check if valid image ref provided
//				useReference = true;
//			} else {
//				#ifdef _DEBUG_MTXCV
//					printf("[Warn] @constructMatrixFromPoints: Provided reference image dimensions do not match returned matrix, continuing with static pixel value.\n\r");
//				#endif
//			}
//			// init return matrix with zeros if requested
//			if(clearBeforeWrite) { imgReturn.setZero(); }
//
//			// Iterate through the edge points and set corresponding intensities in returned matrix
//			for (const EdgePoint* point : edgePoints) {
//				int x = point->coord.x();
//				int y = point->coord.y();
//				// Check if the point is within the matrix bounds
//				if (isValidImageCoord(x, y, cols, rows)) {
//					// Set intensity value at point coord to set-value or the intensity at corresponding imageReference if provided; cast to type of matrix
//					imgReturn(y, x) = static_cast<typename MatrixType::Scalar>((useReference ? (*imageReference)(y, x) : pixelValue));
//				} else {
//					#ifdef _DEBUG_MTXCV
//						//std::cerr << "[Warn] @constructMatrixFromEdgePoints: Edge candidate coord is out of image bounds, continuing next.\n\r";
//						printf("[Warn] @constructMatrixFromPoints: Edge candidate coord is out of image bounds, continuing next.\n\r");
//					#endif
//					fnState = FnState::PROCESSING_WARN;
//					continue; //skip current point coordinate and proceed to next edgePoint
//				}
//			}
//			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
//		}

		/*Print provided matrix to serial in table format
		Parameters:
		- matrix: Input matrix to be printed to serial in table format
		- matrixTitle: String to be printed to serial ahead of matrix table
		- decimalPrecision : precision to print float etc
		- colMinSize : min char spaces to print per col
		Returns: NA */
		template<typename MatrixType>
		static void printMatrixTableFormat(const MatrixType& matrix, const char* matrixTitle, const uint8_t decimalPrecision = 0, const uint8_t colMinSize = 5) {
			// NOTE: std::setw sets the width of the next output field to 8 char, if next value printed < this it will be right-aligned within the 8 char space
			// std::setprecision  is used to round the printed float to 4 decimal places, this should ensure the expected 0->255 float range is printed within the 8 char space (eg 254.1234 is 8 char)
			const Index rows = matrix.rows();
			const Index cols = matrix.cols();

			// Print matrix title and column index key to top row
			printf("Print->%s:\n\ryRow xCol>\n\rV    ", matrixTitle);
			for (int col = 0; col < cols; ++col) {
				printf("%*d", colMinSize, col); //print col index
			}
			//new line and return
			printf("\n\r");
			// Print row index and matrix data
			for (int row = 0; row < rows; ++row) {
				printf("%*d |", 3, row);
				for (int col = 0; col < cols; ++col) {
					printf("%*.*f ", colMinSize - 1, decimalPrecision, roundPrecision(static_cast<float>(matrix(row, col)), decimalPrecision) ); // (colMinSize - 1) to account for extra col to col space in printf
				}
				printf("\n\r");
			}
		}

		/*Function prints matrix to terminal using ASCII font gradient
		Parameters:
		- image: Input image MatrixXf or MatrixXi matrix
		- aspectRatioCompensation: repeats printing char based on aspect ratio to better represent image
		- extendedGradient : select character set, true selects asciiGradientLong[], false selects asciiGradientShort[]
		- pointSequence : when true, all coordinates with intensity > 0 are printed as pointChar (ie 'X'), can be used to print edge sequence without ascii gradient
		Returns: NA */
		template<typename ImageType>
		static void printImageAsciiSerial(const MatrixBase<ImageType>& image, const bool aspectRatioCompensation, const bool extendedGradient = false, const bool pointSequence = false) {
			// REFERENCE:
//			#define ASCII_GRADIENT_LONG_SIZE (29 + NULL_TERMINATOR)
//		    static const char asciiGradientLong[ASCII_GRADIENT_LONG_SIZE] = " _.,-=+:;cba!?0123456789$W#@N";
//		    #define ASCII_GRADIENT_SHORT_SIZE (10 + NULL_TERMINATOR)
//		    static const char asciiGradientShort[ASCII_GRADIENT_SHORT_SIZE] = " .:-=+*#%@";
			const size_t ASCII_GRADIENT_LONG_SIZE = (29 + NULL_TERMINATOR);
			const size_t ASCII_GRADIENT_SHORT_SIZE = (10 + NULL_TERMINATOR);
			const char asciiGradientLong[ASCII_GRADIENT_LONG_SIZE] = " _.,-=+:;cba!?0123456789$W#@N"; //inverse "N@#W$9876543210?!abc;:+=-,._ " ; ASCII char shade gradient array, intensity range is mapped to index
			const char asciiGradientShort[ASCII_GRADIENT_SHORT_SIZE] = " .:-=+*#%@"; //inverse  "@%#*+=-:. "


			static uint32_t callCounter = 0;
			const int gradientSize = (extendedGradient ? ASCII_GRADIENT_LONG_SIZE : ASCII_GRADIENT_SHORT_SIZE);
			const float scalingRatio = (gradientSize - NULL_TERMINATOR) / 256.0f; //number of ASCII char in defined range / byte size range
			const char breakChar = '~'; // break char set as tilda "~" // NOTE: single char are declared with single quotes, string use double quotes
			const char pointChar = 'X';
			const int imgBreakCharCount = 15; //prints x2 this for both sides of start / end flag

			// Define the dimensions of the image
			Index numRows = image.rows();
			Index numCols = image.cols();

			uint8_t aspectRatio = 1;
			if(aspectRatioCompensation){ // when aspectRatioCompensation, result is equivelant to ceil(image.aspectRatio) + 1
				aspectRatio += std::max(1, static_cast<int>(std::ceil(static_cast<float>(numCols) / static_cast<float>(numRows))));
			}
			printf("@ImgAsciiSerial[#%ld] imgBase:[%dx%dpx] aspect(row:col)[1:%d] with asciiGradient:%s.\n\r", callCounter, numCols, numRows, aspectRatio, (extendedGradient ? "Extended" : "Short"));
			printf("%.*s%s%.*s\n\r", imgBreakCharCount, &breakChar, "sImgBegin", imgBreakCharCount, &breakChar);
			// Iterate over each pixel in the image
			for (Index i = 0; i < numRows; ++i) {
				for (Index j = 0; j < numCols; ++j) {
					// Map the pixel intensity value to the corresponding ASCII gradient character index; image intensity rounded to nearest int
					int intensityIndex = std::max(0, std::min((gradientSize - NULL_TERMINATOR - 1), static_cast<int>(scalingRatio * image(i, j) + 0.5f))); // bind ranges to valid index
					char asciiChar = (extendedGradient ? asciiGradientLong[intensityIndex] : asciiGradientShort[intensityIndex]);
					//When printing point sequence, use the pointChar to represent any intensity values > 0
					if(pointSequence && intensityIndex > 0) {
						asciiChar = (pointSequence ? pointChar : asciiChar);
					}
					//print ascii char corresponding to pixel intensity, char is printed aspectRatio times to apply rough compensation for original image dimensions
					for(uint8_t c = 0; c < aspectRatio; ++c) {
						printf("%c", asciiChar);
					}
				}
				printf("\n\r"); //next image row signaled with newLine + carriageReturn
			}
			printf("%*s%s%*s\n\r", imgBreakCharCount, &breakChar, "sImgEnd", imgBreakCharCount, &breakChar); //image finished, leave terminal at carriage return
			callCounter++;
		}


		/*Function to perform convolution using direct matrix kernel approach, with optional extended edge padding (see separableConvolution for faster performance)
		Approach adapted from https://en.wikipedia.org/wiki/Convolution#Discrete_convolution
		Parameters:
		- image: Input image MatrixXf or MatrixXi matrix
		- kernel: Convolution kernel matrix (must be square and odd-numbered)
		- extendImageEdge: Flag indicating whether to pad the image edges with EXTENDED pixel border
		  (edge pixel is repeated into padding, allows convolution of image edge pixels; otherwise kernelPadding sized border truncated in returned image)
		Returns: executed function state */
		template<typename MatrixType, typename KernelType>
		static FnState convolution(MatrixType& image, const KernelType& kernel, const bool extendImageEdge = true) {
			const Index kernel_N = kernel.rows();
			// Check kernel validity, must be NxN square and N must be odd
			if (kernel_N != kernel.cols() || kernel_N % 2 == 0) {
				#ifdef _DEBUG_MTXCV
					//std::cerr << "[Err] @convolution: Kernel matrix must be square and have odd dimensions.\n\r";
					printf("[Err] @convolution: Kernel matrix must be square and have odd dimensions.\n\r");
				#endif
				return FnState::PROCESSING_ERR;
			}
			// Determine kernel dimensions
			const Index kernelPadding = static_cast<Index>((kernel_N - 1) / 2); //this is size of extended border

			// Pad the image if required
			MatrixXf paddedImage;
			if (extendImageEdge) {
				// kernel centre pixel is bound to image border, kernel extends into padding created by the border extension
				paddedImage = padImageExtended(image, kernelPadding); //create image with extended borders (image edge is extended by kernelPadding on all sides)
			} else {
				// kernel border is bound to image border, output is missing kernelPadding pixels around its edge
				paddedImage = image.template cast<float>(); // cast to float for convolution
			}

			// Determine dimensions post padding
			const Index paddedRows = paddedImage.rows();
			const Index paddedCols = paddedImage.cols();
			image.setZero(); // init return matrix to 0, the values have already been saved to paddedImage

			//MatrixXf result = MatrixXf::Zero(paddedRows - (2 * kernelPadding), paddedCols - (2 * kernelPadding));
			// eg. original image 32x24, kernel_N=5, padding=2; resultSize = (no padding)32x24->28x20, (padding)36x28->32x24;
			// Perform convolution, kernel border is bound within calculated paddedRows and paddedCols
			for (Index i = kernelPadding; i < paddedRows - kernelPadding; ++i) {
				for (Index j = kernelPadding; j < paddedCols - kernelPadding; ++j) {
					// Extract image patch
					MatrixXf imagePatch = paddedImage.block(i - kernelPadding, j - kernelPadding, kernel_N, kernel_N);
					// Apply matrix based convolution
					image(i - kernelPadding, j - kernelPadding) = static_cast<typename MatrixType::Scalar>((imagePatch.array() * kernel.array()).sum());
				}
			}
			return FnState::OK;
		}


		/*Function to perform faster convolution using the seperable Row and Column vectors of a square kernel matrix, with optional extended edge padding
		 * Approach reference https://towardsdatascience.com/a-basic-introduction-to-separable-convolutions-b99ec3102728; https://en.wikipedia.org/wiki/Separable_filter
		Parameters:
		- image: Input image MatrixXf or MatrixXi matrix
		- rowVector: Seperable convolution kernel row vector (must be from a square kernel and odd-number size)
		- colVector: Seperable convolution kernel column vector (must be from a square kernel and odd-number size)
		- extendImageEdge: Flag indicating whether to pad the image edges with EXTENDED pixel border
		  (edge pixel is repeated into padding, allows convolution of image edge pixels; otherwise kernelPadding sized border truncated in returned image)
		Returns: executed function state */
		template<typename MatrixType>
		static FnState separableConvolution(MatrixType& image, const RowVectorXf& rowVector, const VectorXf& colVector, const bool extendImageEdge = true) {
			// Determine dimensions post padding
			const Index kernelColVectSize = colVector.size();
			const Index kernelRowVectSize = rowVector.size();

			// Check kernel validity, must be NxN square and N must be odd
			if (kernelColVectSize != kernelRowVectSize || kernelColVectSize % 2 == 0) {
				#ifdef _DEBUG_MTXCV
					printf("[Err] @convolution: Kernel row and col vectors must be equal size and have odd dimensions.");
					printf(" RowVect: %u, ColVect: %u.\n\r", static_cast<int>(kernelRowVectSize), static_cast<int>(kernelColVectSize));
				#endif
				return FnState::PROCESSING_ERR;
			}
			// Determine kernel dimensions
			const Index kernelPadding = static_cast<Index>((kernelColVectSize - 1) / 2); //this is size of extended border

			// Pad the image if required
			MatrixXf paddedImage;
			if (extendImageEdge) {
				// kernel centre pixel is bound to image border, kernel extends into padding created by the border extension
				paddedImage = padImageExtended(image, kernelPadding); //create image with extended borders (image edge is extended by kernelPadding on all sides)
			} else {
				// kernel border is bound to image border, output is missing kernelPadding pixels around its edge
				paddedImage = image.template cast<float>(); // cast to float for convolution
			}
			// Determine dimensions post padding
			const Index paddedRows = paddedImage.rows();
			const Index paddedCols = paddedImage.cols();
			image.setZero(); // init return matrix to 0, the values have been stored in paddedImage

			// Temporary matrix to store intermediate result after col-wise convolution
			MatrixXf intermediate(paddedRows - (2 * kernelPadding), paddedCols);

			// Convolve along cols
			/*  eg assuming 3x3 seperated kernel on 5x4 image; cursor vector centre, progress each col, row idx 1 to 2
				[>][0][0][0][0]
				[>][0][0][0][0]
				[>][0][0][0][0]
				[0][0][0][0][0]*/
			for (Index jRow = kernelPadding; jRow < paddedRows - kernelPadding; ++jRow) {
				for (Index iCol = 0; iCol < paddedCols; ++iCol) {
					//Col vector, multiple rows and one column.
					VectorXf colPatch = paddedImage.col(iCol).segment(jRow - kernelPadding, kernelColVectSize);
					// Convolve along cols
					float colResult = (colPatch.array() * colVector.array()).sum();
					// Store the result in the intermediate matrix
					intermediate(jRow - kernelPadding, iCol) = colResult;
				}
			}

			// Convolve along rows
			/*  eg assuming 3x3 seperated kernel on 5x4 image; cursor vector centre, progress each row, col idx 1 to 3; intermediate image convolution:
				[V][V][V][0][0]
				[0][0][0][0][0]*/
			for (Index iCol = kernelPadding; iCol < paddedCols - kernelPadding; ++iCol) {
				for (Index jRow = 0; jRow < paddedRows - (2 * kernelPadding); ++jRow) {
					// Row vector, one row and multiple columns.
					RowVectorXf rowPatch = intermediate.row(jRow).segment(iCol - kernelPadding, kernelRowVectSize);
					// Convolve along rows
					float rowResult = (rowPatch.array() * rowVector.array()).sum();
					// Store the final result in the output image matrix
					image(jRow, iCol - kernelPadding) = static_cast<typename MatrixType::Scalar>(rowResult);
				}
			}
			return FnState::OK;
		}

//		/*Function to decompose an MxN kernel matrix into two 1D vectors of Mx1 (M rows, 1 col) and 1xN (1 row, N cols) for separable convolution (can lead to faster convolution vs raw matrix).
//		Equations and approach adapted from: https://en.wikipedia.org/wiki/Singular_value_decomposition; https://tmp.mosra.cz/eigen-docs/classEigen_1_1JacobiSVD.html; https://bartwronski.com/2020/02/03/separate-your-filters-svd-and-low-rank-approximation-of-image-filters/
//		Parameters:
//		- kernelMatrix: Input MxN kernel to decompose into row and column vector
//		- colVector: Returned colVector decomposed from kernelMatrix of size Mx1
//		- rowVector: Returned rowVector decomposed from kernelMatrix of size 1xN
//		Returns: executed function state */
//		template<int Size>
//		static FnState decomposeKernelMatrix(const Matrix<float, Size, Size>& kernelMatrix, Vector<float, Size>& colVector, RowVector<float, Size>& rowVector) {
//		//TBD another option avoiding dynamic mtx: template<int M, int N> // const Matrix<float, M, N>& kernelMatrix, Matrix<float, M, 1>& colVector, Matrix<float, 1, N>& rowVector
//			/*NOTE: VectorXf: has multiple rows and one column.
//					RowVectorXf: has one row and multiple columns.
//				If only the first singular value is non-zero (or the others are very small), the kernel is separable.*/
//			const float rank1Tolerance = 1e-5f; //tolerance for the value of the second singularValue; should be lower than this for accurate separable matrix
//
//			// Check if the input matrix is valid, square and odd
//			if (kernelMatrix.cols() <= 0 || kernelMatrix.rows() <= 0) {
//				#ifdef _DEBUG_MTXCV
//					printf("[Err] @decomposeFilterMatrix: Input kernel matrix is empty or not NxN square.\n\r");
//				#endif
//				return FnState::PROCESSING_ERR;
//			}
//
//			// Calculate the SVD decomposition of the filter matrix
//			JacobiSVD<MatrixXf> svd(kernelMatrix.template cast<float>(), ComputeThinV | ComputeThinU); // JacobiSVD constructor to perform decomposition of input matrix // ComputeFullU | ComputeFullV
//			// the singularValues are returned in descending order, the number returned indicates the matrix rank
//			// for a matrix separable to two vectors only the first singularValue is dominant (ie matrix is rank 1).
//			// as the singularVal0 coefficient is multiplied into both the returned colVector and rowVector, we take the sqrt so the recombined vectors correctly resolve back to the original kernelMatrix
//			float singularVal0_sqrt = sqrtf(svd.singularValues().coeff(0)); // extract the dominating singularValue from first index;
//			float singularVal1 = svd.singularValues().coeff(1); // extract the secondary singularValue to check accuracy of decomposition (should be 0 or very small to accuratly reconstruct kernelMatrix)
//
//			colVector = svd.matrixU().col(0); //store unscaled U vector component (left-singular vector)
//			rowVector = svd.matrixV().col(0); //store unscaled V vector component (right-singular vector)
//
//			//rescale vectors based on singularVal0 (sqrt of singularVal0 to account it being applied to both vectors seperatly, ie initial coeff is squared)
//			colVector *= singularVal0_sqrt;
//			rowVector *= singularVal0_sqrt;
//
//			#ifdef _DEBUG_MTXCV
//				//Test reconstruction accuracy
//				MatrixXf reconstructedMatrix = colVector * rowVector;
//				printf("@decomposeFilterMatrix: singularValue0[%f] singularValue1[%f]\n\r", svd.singularValues().coeff(0), svd.singularValues().coeff(1));
//				printf("kernelMatrix input:\n\r"); printMatrix(kernelMatrix);
//				printf("->colVector:\n\r"); printMatrix(colVector);
//				printf("->rowVector: "); printMatrix(rowVector);
//				printf("reconstructedMatrix:\n\r"); printMatrix(reconstructedMatrix);
//			#endif
//
//			// Check if the input matrix is rank deficient (ie the matrix should be rank 1, meaning the second singularValues is 0 or very small)
//			if (singularVal1 > rank1Tolerance) { //tolerance of rank1 categorisation
//				#ifdef _DEBUG_MTXCV
//					printf("[Warn] @decomposeFilterMatrix: Input matrix is not rank one. Returning low-rank approximation.\n\r");
//				#endif
//				return FnState::PROCESSING_WARN;
//			}
//
//			return FnState::OK;
//		}

//		/*Function to generate Gaussian NxN filter kernel
//		Sigma calculation adapted from: https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gac05a120c1ae92a6060dd0db190a61afa
//		Parameters:
//		- Size: N dimension of square NxN kernelReturn provided
//		- kernelReturn: float matrix to populate and return
//		Returns: executed function state */
//		template<int Size>
//		static FnState generateGaussianKernel(Eigen::Matrix<float, Size, Size>& kernelReturn) {
//			const Index kernel_N = kernelReturn.rows(); //get the size of passed kernel from template
//			// Check if the kernel size is valid (positive odd integer)
//			if (kernel_N != kernelReturn.cols() || kernel_N % 2 == 0) {
//				#ifdef _DEBUG_MTXCV
//					//std::cerr << "[Err] @generateGaussianKernel: Kernel size must be a positive odd number.\n\r";
//					printf("[Err] @generateGaussianKernel: Kernel size must be odd numberd and square, exiting function.\n\r");
//				#endif
//				return FnState::PROCESSING_ERR; // Return an empty matrix to indicate failure
//			}
//			// Calculate patch padding, eg pixel distance from kernel centre to its edge; eg 3x3=1, 5x5=2, 7x7=3 etc
//			int halfSize = (Size - 1) / 2;
//
//			// Calculate sigma using the recommended approach in Gaussian blur
//			const float sigmaMultiplier = 0.3f; // Multiplier factor for sigma calculation
//			const float sigmaBase = 0.8f;       // Base value for sigma calculation
//
//			// sigma calculation adapted from: https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gac05a120c1ae92a6060dd0db190a61afa
//			float sigma = sigmaMultiplier * ((kernel_N - 1) * 0.5f - 1) + sigmaBase; // [5x5 = 1.1 sigma]
//			sigma = std::max(sigma, 0.1f); // Bind min value to ensure sigma does not create div 0
//
//			const float scalingConst = 1.0f; // parameter represents height of the distribution peak
//			float kernelSum = 0.0f; // tracks sum of kernel values for normalisation
//
//			kernelReturn.setZero(); // clean kernel values to 0
//			//Populate kernel centre pixel seperatly as it is skipped in following loop
//			kernelReturn(halfSize, halfSize) = scalingConst;
//			kernelSum += scalingConst;
//
//			//Calculate kernel values for one quarter of kernel patch (since Gaussian kernel is rotationally symmetric)
//			// eg. let quadrants of kernal be:
//			//  [Q2][Q1]
//			//  [Q3][Q4]
//			for (int x = 0; x <= halfSize; ++x) { // loop all columns including centre pixel
//				for (int y = 0; y < halfSize; ++y) { // loop all rows excluding centre pixel
//					int xDistance = x - halfSize; // x offset from kernel (0,0)
//					int yDistance = y - halfSize; // y offset from kernel (0,0)
//
//					// Calculate Gaussian value based on distance from kernel center
//					float value = scalingConst * exp(-(xDistance * xDistance + yDistance * yDistance) / (2 * sigma * sigma));
//
//					// apply value to kernel at current position
//					kernelReturn(x, y) = value; // loadsInto:[Q2], or [Q2][Q1] boundary when x == halfSize
//					// mirror across both axis diagonally
//					kernelReturn(kernel_N - 1 - x, kernel_N - 1 - y) = value; // loadsInto:[Q4], or [Q3][Q4] boundary when x == halfSize
//
//					//Check if we are at the centre pixel
//					if(x == halfSize){
//						// when x == halfSize, INVERT COORD to correctly populate centre pixels along middle of kernel
//						kernelReturn(y, kernel_N - 1 - x) = value; // loads [Q2][Q3] boundary when x == halfSize
//						kernelReturn(kernel_N - 1 - y, x) = value; // loads [Q1][Q4] boundary when x == halfSize
//					} else {
//						kernelReturn(kernel_N - 1 - x, y) = value; // Mirror horizontally across y-axis; loadsInto:[Q1]
//						kernelReturn(x, kernel_N - 1 - y) = value; // Mirror vertically across x-axis; loadsInto:[Q3]
//					}
//					// Update sum (multiplied to account mirrored cells as well)
//					kernelSum += 4 * value;
//				}
//			}
//			// Normalise the kernel by dividing each element by the sum of all elements
//			kernelReturn /= kernelSum;
//			return FnState::OK; // Return the generated Gaussian kernel
//		}

//		/* Function to generate Gaussian NxN filter kernel as seperable Row and Col vectors
//		Parameters:
//		- Size: N dimension of square NxN kernelReturn to produce seperable vectors for
//		- colVectorReturn: returned gaussian kernel decomposed seperable col vector
//		- rowVectorReturn: returned gaussian kernel decomposed seperable row vector
//		Returns: executed function state */
//		template<int Size>
//		static FnState generateGaussianKernelSeperated(Vector<float, Size>& colVectorReturn, RowVector<float, Size>& rowVectorReturn) {
//			FnState fnState = FnState::INITIALISED;
//			//Create temp gaussian matrix of Size x Size
//			Matrix<float, Size, Size> gaussianKernel;
//			gaussianKernel.setZero();
//			fnState = evalFnStates(fnState, generateGaussianKernel<Size>(gaussianKernel));
//
//			//Decompose kernel into row and column vectors
//			fnState = evalFnStates(fnState, decomposeKernelMatrix(gaussianKernel, colVectorReturn, rowVectorReturn));
//
//			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
//		}

//		/*Function calculates the histogram for a passed image, the results are populated to the passed histogram array; NO SCALING OCCURS, values outside byte range are bound to uint8_t range
//		Parameters:
//		- image: Input image to calculate histogram over
//		- histogram: returned histogram associated with provided image, index of array is corresponding intensity level, and its value is the number of pixels found in image at that intensity
//		Returns: NA */
//		template<typename MatrixType>
//		static void getImageHistogram(const MatrixType& image, Array256byte& histogram) {
//			const Index numBins = histogram.size(); //get size of histogram, will return 256 when using typedef Array256byte, eg 8-bit image is 0-255 = 256 bin locations
//			histogram.setZero(numBins); //clear array to 0
//			const Index imgRows = image.rows();
//			const Index imgCols = image.cols();
//			// for each pixel in the image
//			for (Index j = 0; j < imgRows; ++j) { //for each y_row in image
//				for (Index i = 0; i < imgCols; ++i) { //for each x_col in image y_row
//					// get intensity at pixel, round to nearest integer and bind to byte 0-255 range
//					int intensityIdx = std::max(0, std::min(static_cast<int>(numBins) - 1, static_cast<int>(0.5f + image(j, i))) );
//					// increment value +1 in histogram array at index corresponding to pixel intensity value
//					histogram(intensityIdx)++;
//				}
//			}
//		}

//		/*Function to equalise the histogram of a grayscale image (improves edge contrast); input image intensity should be pre-scaled to 0-255 byte range
//		Parameters:
//		- image: Input image reference is equalised and returned
//		Returns: executed function state */
//		template<typename MatrixType>
//		static FnState equaliseHistogram(MatrixType& image) {
//			FnState fnState = FnState::INITIALISED;
//			// Compute histogram; each index of histogram vector corresponds to a pixel 0-255 intensity,
//			// the value stored at an index is the number of pixels in the image at that intensity
//			Array256byte histogram;
//			getImageHistogram(image, histogram); //populate histogram based on image pixel intensities
//			const int numBins = static_cast<int>(histogram.size());  // byte range 0->255 = 256 elements
//			#ifdef _DEBUG_MTXCV
//				//std::cout << "@equalizeHist: pixelCount:[" << pixelCount << "] histogramSum:[" << histogram.sum() << "].\n\rcdf:\n\r" << cdf << "\n\r";
//				// printf("@equalizeHist: histogramBins:[%d] histogramSum:[%d].\n\rhistogram:\n\r", numBins, static_cast<int>(histogram.cast<int>().sum()));
//				// printMatrix(histogram);
//			#endif
//			// Calculate Cumulative Distribution Function (CDF)
//			// (ie represents cumulative probability distribution of pixel intensities in the image)
//			Array<uint16_t, 256, 1> cdf;
//			cdf.setZero();
//			cdf(0) = histogram(0); // prime cumulative sum by populating first value at index intensity level 0 (black)
//			for (Index i = 1; i < numBins; ++i) { // for index 1 to
//				//perform cumulative sum of intensities at and below current intensity value
//				cdf(i) = cdf(i - 1) + histogram(i); // cdf array contains the cumulative sum of histogram values up to each intensity level
//			}
//			// record image dimansions and pixel count
//			const Index imgRows = image.rows();
//			const Index imgCols = image.cols();
//			uint16_t pixelCount = static_cast<uint16_t>(imgRows * imgCols);
//
//			#ifdef _DEBUG_MTXCV
//				//std::cout << "@equalizeHist: pixelCount:[" << pixelCount << "] histogramSum:[" << histogram.sum() << "].\n\rcdf:\n\r" << cdf << "\n\r";
//				// printf("@equalizeHist: pixelCount:[%d].\n\rcdf:\n\r", pixelCount);
//				// printMatrix(cdf);
//			#endif
//
//			// Normalise CDF array, lowest non-zero value is clamped to 0 and therefore subtracted from normalisation
//			// Find the min non-zero intensity in image; using Eigen version of ternary element wise operator; cdf <= 0 are evaluated as int max, find lowest value from this adjusted list
//			uint16_t cdf_minVal = (cdf > 0).select(cdf, UINT8_MAX).minCoeff();
//			if(cdf_minVal == pixelCount) {
//				cdf_minVal -= 1; // avoid div 0
//				fnState = FnState::PROCESSING_WARN; //flag function warning
//				#ifdef _DEBUG_MTXCV
//					printf("[Warn] @equalizeHist: Cumulative Distribution Function min value was adjusted to avoid div0, continuing.\n\r");
//				#endif
//			}
//			// equation adapted from: https://en.wikipedia.org/wiki/Histogram_equalization
//			// For cdf values > 0, normalise to 0->1 range based on distribution of intensities in image
//			//Array256float cdf_norm = ((cdf.cast<float>() - cdf_minVal) / (pixelCount - cdf_minVal)).cast<float>();
//			Array<float, 256, 1> cdf_norm = ((cdf.cast<float>() - static_cast<float>(cdf_minVal)) / (static_cast<float>(pixelCount) - static_cast<float>(cdf_minVal))).cast<float>();
//			//Array<float, numBins, 1> cdf_norm = ((cdf > 0).select(static_cast<float>(cdf - cdf_minVal) / (pixelCount - cdf_minVal), 0)).cast<float>();
//
//			#ifdef _DEBUG_MTXCV
//				//std::cout << "cdf_minVal:[" << cdf_minVal << "] nonZeros:[" << cdf.nonZeros() <<"]\n\r.";
//				//std::cout << "cdf_norm array:\n\r" << cdf_norm << "\n\r";
//				// printf("cdf_minVal:[%u] nonZeros:[%u]\n\r.cdf_norm array:\n\r", cdf_minVal, static_cast<uint16_t>(cdf.nonZeros()));
//				// printMatrix(cdf_norm);
//			#endif
//
//			// Perform histogram equalization
//			//MatrixXf equalizedImage(imgRows, imgCols);
//			for (Index j = 0; j < imgRows; ++j) {
//				for (Index i = 0; i < imgCols; ++i) { //for each pixel in image
//					// round intensity to byte range and convert back to datatype of the passed image matrix
//					int intensity = std::max(0, std::min(numBins - 1, static_cast<int>(image(j, i) + 0.5f))); // Ensure index is bound within vector size
//					image(j, i) = static_cast<typename MatrixType::Scalar>(cdf_norm(intensity) * 255.0f); // multiply distribution by uint8_t to scale 0->1 to 0->255, cast to type if image matrix (assumed float or integer)
//				}
//			}
//			//printMatrixTableFormat(image, "equalisedHistogram:\0");
//			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
//		}

		/*Function to compute gradient magnitude and direction of pixel intensity for canny edge detection
		Approach adapted from https://en.wikipedia.org/wiki/Canny_edge_detector
		Parameters:
		- image: Input image to calculate gradient
		- gradMag: Returned matrix of gradient magnitudes (vector radius: sqrt(x^2 + y^2) )
		- gradDir: Returned matrix of gradient direction (vector angle in radians, +x axis down is positive)
		Returns: executed function state */
		template<typename MatrixType>
		static FnState computeGradient(const MatrixType& image, MatrixType& gradMag, MatrixType& gradDir) {
			FnState fnState = FnState::INITIALISED;
			const Index sobel_N = 3;
			// Sobel edge detection kernels
			const MtxKernelfloat<sobel_N> sobelX = (MtxKernelfloat<sobel_N>() << //setup such that x increases left to right ->
				-1.0f, 0.0f, 1.0f,
				-2.0f, 0.0f, 2.0f,
				-1.0f, 0.0f, 1.0f).finished();
			const MtxKernelfloat<sobel_N> sobelY = sobelX.transpose().rowwise().reverse(); //rotate CW 90 degrees
			// const MtxKernelfloat<sobel_N> sobelY = (MtxKernelfloat<sobel_N>() << // EXAMPLE OF sobelY MATRIX: setup such that y increases top to bottom V
			//     -1.0f, -2.0f, -1.0f,
			//     0.0f,  0.0f,  0.0f,
			//     1.0f,  2.0f,  1.0f).finished();  //MtxImgfloat::Zero();

			// printMatrixTableFormat(image, "image[1]:\0");
			// Compute gradients in image using Sobel kernels
			MatrixType gradX = image; // deep copy
			MatrixType gradY = image; // deep copy
			#ifdef _DEBUG_MTXCV
				// printMatrixTableFormat(gradX, "gradX[1]:\0");
				// printMatrixTableFormat(gradY, "gradY[1]:\0");
			#endif
			//perform convolution in x and y direction
			fnState = evalFnStates(fnState, convolution(gradX, sobelX, true)); //perform convolution x-gradient and track err if it occured
			fnState = evalFnStates(fnState, convolution(gradY, sobelY, true)); //perform convolution y-gradient and track err if it occured
			#ifdef _DEBUG_MTXCV
				// printMatrixTableFormat(gradX, "gradX[2]:\0");
				// printMatrixTableFormat(gradY, "gradY[2]:\0");
			#endif
			// Compute gradient magnitude and direction
			gradMag = (gradX.array().square() + gradY.array().square()).sqrt(); // equivelant sqrt(x^2 + y^2)
			// element-wise binary operation between matrices using lambda function; used here as Eigen missing direct atan2 class funct
			gradDir = gradY.binaryExpr(gradX, [](float y, float x) { return std::atan2(y, x); });

			#ifdef _DEBUG_MTXCV
				//printf("@computeGradient\n\r");
				// printf("{gradX}:[%u, %u]\n\r", static_cast<uint16_t>(gradX.cols()), static_cast<uint16_t>(gradX.rows())); printMatrix(gradX);
				// printf("{gradY}:[%u, %u]\n\r", static_cast<uint16_t>(gradY.cols()), static_cast<uint16_t>(gradY.rows())); printMatrix(gradY);
				//printf("{gradMag}:[%u, %u]\n\r", static_cast<uint16_t>(gradMag.cols()), static_cast<uint16_t>(gradMag.rows())); printMatrix(gradMag);
				// printf("{gradDir}:[%u, %u]\n\r", static_cast<uint16_t>(gradDir.cols()), static_cast<uint16_t>(gradDir.rows())); printMatrix(gradDir);
			#endif

			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
		}

		/*Function applies the Non-Maximum Suppression technique to refine edge detection results with linear interpolation between pixels when the gradient direction intersects two adjacent pixels coords.
		Approach adapted from https://en.wikipedia.org/wiki/Canny_edge_detector
		Parameters:
		- gradientMagnitude: Matrix representing gradient magnitude of the image
		- gradientDirection: Matrix representing gradient direction of the image
		- suppressed: Matrix representing the suppressed gradient magnitude after Non-Maximum Suppression is applied
		Returns: executed function state */
		template<typename MatrixType>
		static FnState nonMaxSuppression(const MatrixType& gradientMagnitude, const MatrixType& gradientDirection, MatrixType& suppressedReturn) {
			/* Notes on gradient directions:
			The gradient matrix frame is defined with increasing axis x->(left to right), y->(top to bottom); angle points to largest gradient change,
			and ranges from -M_PI to M_PI, the edge is prependicular to the gradient direction, and rotationally symmetric
			ie  -M_PI/2 || M_PI/2 indicates north<>south || south<>north gradient, potential edge perpendicular to gradient is horizontal;
				0 || M_PI || -M_PI indicates east<>west || west<>east gradient, potential edge perpendicular to gradient is vertical;
				...and other combinations forming diagonals */
			FnState fnState = FnState::INITIALISED;

			// TBD CANDIDATE FOR GLOBAL VAR
			constexpr float trigQuadrantOffset = static_cast<float>(M_PI) * 0.25f; // quadrant offset to align angle ranges with adjacent pixel locations
			constexpr float trig45 =  static_cast<float>(M_PI) * 0.25f;
			constexpr float trig90 =  static_cast<float>(M_PI) * 0.5f;
			constexpr float trig135 = static_cast<float>(M_PI) * 0.75f;

			const Index rows = gradientMagnitude.rows();
			const Index cols = gradientMagnitude.cols();
			//Validate passed matrix dimensions
			bool rowCheck = (rows == gradientDirection.rows() && gradientDirection.rows() == suppressedReturn.rows());
			bool colCheck = (cols == gradientDirection.cols() && gradientDirection.cols() == suppressedReturn.cols());
			// Ensure both magnitude and angle gradient matrix are same size
			if (!(rowCheck && colCheck)) {
				#ifdef _DEBUG_MTXCV
					//std::cerr << "[Err] @nonMaxSuppression: Gradient magnitude and direction matrix must have the same dimensions.\n\r";
					printf("[Err] @nonMaxSuppression: Gradient magnitude and direction matrix must have the same dimensions.\n\r");
				#endif
				return FnState::PROCESSING_ERR; // return error
			}
			// init the suppressed matrix to return
			// NOTE: ## image returns with one px black border when full px range evaluated without border extension, TBD ##
			suppressedReturn.setZero();

			//offset from current px to interpolation candidate A
			int8_t pxA_xOffset = 0;
			int8_t pxA_yOffset = 0;
			float pxA_val = 0.0f; // candidate A px magnitude in image
			//offset from current px to interpolation candidate B
			int8_t pxB_xOffset = 0;
			int8_t pxB_yOffset = 0;
			float pxB_val = 0.0f; // candidate B px magnitude in image

			float weight = 1.0f; //init weighting ratio for interpolation contribution of pixel A to B as % ratio
			float interpolatedDir1 = 0.0f; //interpolated magnitude of candidate px A and B
			float interpolatedDir2 = 0.0f; //interpolated magnitude of candidate px A and B (reversed vector direction, ie +-180deg)

			// Perform non-maximum suppression; bound within image with 1px border offset from image edge (size of assessed 3x3 kernel)
			for (Index yRow = 1; yRow < rows - 1; ++yRow) {
				for (Index xCol = 1; xCol < cols - 1; ++xCol) {
					float mag = static_cast<float>(gradientMagnitude(yRow, xCol));
					float dir = static_cast<float>(gradientDirection(yRow, xCol));
					// bind gradient angle to -M_PI -> M_PI range (validation check as should already be bound from prev atan2 calc)
					while (dir > M_PI) {
						dir -= 2.0f * static_cast<float>(M_PI);
					}
					while (dir < -M_PI) {
						dir += 2.0f * static_cast<float>(M_PI);
					}
					// Calculate offset quadrant to identify candidate neighboring pixels for sub-pixel interpolation calc
					/*quadrant	range1	    range2
						0	    0 to 44	    -180 to -136
						1	    45 to 89	-135 to -91
						2	    90 to 134	-90 to -46
						3	    135 to 179	-45 to -1   */
					uint8_t offsetQuadrant = static_cast<uint8_t>(std::floor(dir / trigQuadrantOffset)) % 4; // get offset quadrant from magnitude angle
					// Determine offsets based on the offset quadrant identfied
					switch (offsetQuadrant) {
						case 0: // two vector gradient enpoint values are interpolation: E <~> SE pixel values; and W <~> NW pixel values
							// relative candidate pixels A[1,0], B[1,1], and inverse direction A[-1,0], B[-1,-1]
							pxA_xOffset = 1; pxA_yOffset = 0;
							pxB_xOffset = 1; pxB_yOffset = 1;
							weight = std::tan(dir); //TBD these tan calcs for weight can probably be hard baked to a lookup given the small anticipated 0->45deg range
							break;
						case 1: // two vector gradient enpoint values are interpolation: SE <~> S pixel values; and NW <~> N pixel values
							// relative candidate pixels A[1,1], B[0,1], and inverse direction A[-1,-1], B[0,-1]
							pxA_xOffset = 1; pxA_yOffset = 1;
							pxB_xOffset = 0; pxB_yOffset = 1;
							weight = std::tan(dir - trig45); //subtract 45deg to bind tan to valid range
							break;
						case 2: // two vector gradient enpoint values are interpolation: S <~> SW pixel values; and N <~> NE pixel values
							// relative candidate pixels A[0,1], B[-1,1], and inverse direction A[0,-1], B[1,-1]
							pxA_xOffset = 0; pxA_yOffset = 1;
							pxB_xOffset = -1; pxB_yOffset = 1;
							weight = std::tan(dir - trig90); //subtract 90deg to bind tan to valid range
							break;
						case 3: // two vector gradient enpoint values are interpolation: SW <~> W pixel values; and NE <~> E pixel values
							// relative candidate pixels A[-1,1], B[-1,-1], and inverse direction A[1,-1], B[1,1]
							pxA_xOffset = -1; pxA_yOffset = 1;
							pxB_xOffset = -1; pxB_yOffset = -1;
							weight = std::tan(dir - trig135); //subtract 135deg to bind tan to valid range
							break;
						default: //TBD correct handeling undefined angle, should be impossible
							pxA_xOffset = 0; pxA_yOffset = 0;
							pxB_xOffset = 0; pxB_yOffset = 0;
							#ifdef _DEBUG_MTXCV
								//std::cerr << "[Warn] @nonMaxSuppression: Gradient angle resolved an undefined quadrant.\n\r";
								printf("[Warn] @nonMaxSuppression: Gradient angle resolved an undefined quadrant.\n\r");
								fnState = FnState::PROCESSING_WARN;
							#endif
							break;
					}
					// Calculate pixel interpolation for gradient direction #1
					pxA_val = static_cast<float>(gradientMagnitude(yRow + pxA_yOffset, xCol + pxA_xOffset));
					pxB_val = static_cast<float>(gradientMagnitude(yRow + pxB_yOffset, xCol + pxB_xOffset));
					// interpolated pixel intensity of contributing pixels A and B
					interpolatedDir1 = interpolateLinear(pxA_val, pxB_val, weight);

					// Calculate pixel interpolation for gradient direction #2
					pxA_val = static_cast<float>(gradientMagnitude(yRow - pxA_yOffset, xCol - pxA_xOffset));
					pxB_val = static_cast<float>(gradientMagnitude(yRow - pxB_yOffset, xCol - pxB_xOffset));
					// interpolated pixel intensity of inverted contributing pixels A and B
					interpolatedDir2 = interpolateLinear(pxA_val, pxB_val, weight);

					// Check if the magnitude at the central pixel is greater than the interpolated values in both directions
					if (mag > interpolatedDir1 && mag > interpolatedDir2) {
						suppressedReturn(yRow, xCol) = static_cast<typename MatrixType::Scalar>(mag);
					}
				}
			}
			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
		}

		// BELOW IS TEMP helper fuction to reduce memory usage in main, it simply combinees the functionality of MtxCV::cannyEdgeDetection() and MtxCV::edgeTrackingByHysteresis()
		template<typename MatrixType>
		static void cannyTrackingHelperFn(const MatrixType& image, vector<EdgePoint>& edgeCandidates, vector<EdgePoint*>& edgeSequenceListReturn, const float& gradientThreshRatio_low, const float& gradientThreshRatio_high, const uint8_t& setWeakMaskTo = 128, const uint8_t& setStrongMaskTo = 255, const size_t& minSequenceLength = 10) {
			constexpr int rows = MatrixType::RowsAtCompileTime;
			constexpr int cols = MatrixType::ColsAtCompileTime;
			Matrix<int, rows, cols> edgeMask; // MAIN EDGE MASK FRAME

			cannyEdgeDetection(image, edgeCandidates, gradientThreshRatio_low, gradientThreshRatio_high, &edgeMask, setWeakMaskTo, setStrongMaskTo);

			edgeTrackingByHysteresis(edgeMask, edgeCandidates, edgeSequenceListReturn, minSequenceLength, setWeakMaskTo, setStrongMaskTo);
		}

		/*Canny edge detection uses Sobel operator to identified gradient changes in image, non-maximum edge candidates are suppressed to thin lines, the coords of these edge candidates are returned in vector form, aswell as the evaluated edge mask
		Approach adapted from https://en.wikipedia.org/wiki/Canny_edge_detector
		NOTE: TBD, currently companion function calculateOtsuThreshold calculates thresholdHigh as a pixel intensity value in the image, while the current implementation of cannyEdgeDetection takes a ratio of the max gradient found in the image.
		Parameters:
		- image: Input image matrix
		- edgeCandidates: Returned vector of edge points identified in image
		- gradientThreshRatio_low: Ratio 0->1 of the dynThreshHigh used to calculate dynThreshLow, denotes the lower bound of the gradient value that qualifies as an Weak edge
		- gradientThreshRatio_high: Ratio 0->1 of the images max gradient value used to calculate the dynThreshHigh, denotes the lower bound of the gradient value that qualifies as an Strong edge
		- imageEdgeMaskReturn: if provided, returned image mask is updated with weak and strong edge candidates in image, intensity values are set by setWeakEdgeAs and setStrongEdgeAs
		- setWeakEdgeAs: The value to set an identified Weak edge in the returned image edge mask.
		- setStrongEdgeAs: The value to set an identified Strong edge in the returned image edge mask.
		Returns: executed function state */
		template<typename MatrixType, typename EdgeMaskType>
		static FnState cannyEdgeDetection(const MatrixType& image, vector<EdgePoint>& edgeCandidates, const float& gradientThreshRatio_low, const float& gradientThreshRatio_high, EdgeMaskType *imageEdgeMaskReturn = nullptr, const uint8_t& setWeakMaskTo = 128, const uint8_t& setStrongMaskTo = 255) {
			FnState fnState = FnState::INITIALISED;
//			const Index rows = image.rows();
//			const Index cols = image.cols();

			// Fixed-size matrices
			constexpr int rows = MatrixType::RowsAtCompileTime;
			constexpr int cols = MatrixType::ColsAtCompileTime;

			// Compute gradient magnitude and direction
	        // Use fixed-size matrices for memory efficiency
	        Matrix<float, rows, cols> gradMag;
	        Matrix<float, rows, cols> gradDir;
	        Matrix<float, rows, cols> suppressed;
	        // init matrices to 0
			gradMag.setZero();
			gradDir.setZero();
			suppressed.setZero();
//			MatrixXf gradMag(rows, cols); //init gradient image
//			MatrixXf gradDir(rows, cols);
//			MatrixXf suppressed(rows, cols);
			// DEBUG NOTE: TO BEAT: storage ovf by 4940 bytes

			//Calculate image pixel gradient magnitude and direction
			fnState = evalFnStates(fnState, computeGradient(image, gradMag, gradDir)); //calculate image gradients and track err if it occured
			//fnState = evalFnStates(fnState, computeGradient<MatrixXf>(image.template cast<float>(), gradMag, gradDir));

			// Perform non-maximum suppression for pixels with weak gradient linking
			fnState = evalFnStates(fnState, nonMaxSuppression(gradMag, gradDir, suppressed)); //perform non-max edge suppression and track err if it occured

			edgeCandidates.clear();

			// init dynamic thresholds, values calculated based on success of previous functions
			float dynThreshHigh;
			float dynThreshLow;
			if(fnState == FnState::OK){
				dynThreshHigh = static_cast<float>(suppressed.maxCoeff()) * gradientThreshRatio_high;
				dynThreshLow = dynThreshHigh * gradientThreshRatio_low;
			} else { // TBD this is a non-optimal catch all as dynThresh is evaluated against gradient of suppressed, not pixel intensity
				dynThreshHigh = static_cast<float>(setStrongMaskTo);
				dynThreshLow = static_cast<float>(setWeakMaskTo);
			}
			// determine if a valid edgeMask return has been requested
			bool returnMaskRequest = false; // track if a returned image edge mask was requested
			if(imageEdgeMaskReturn != nullptr) {
				if(imageEdgeMaskReturn->rows() == rows && imageEdgeMaskReturn->cols() == cols) {
					imageEdgeMaskReturn->setZero();
					returnMaskRequest = true;
				} else {
					#ifdef _DEBUG_MTXCV
						printf("[Warn] @cannyEdgeDetection: Provided reference image dimensions do not match provided edge mask, continuing without edge mask return.\n\r");
					#endif
					fnState = evalFnStates(fnState, FnState::PROCESSING_WARN);
				}
			}
			#ifdef _DEBUG_MTXCV
				printf("@cannyEdgeDetection:\n\rsuppressedMax[%.3f] dynThreshLow:[%.3f] dynThreshHigh:[%.3f].\n\r", static_cast<float>(suppressed.maxCoeff()), dynThreshLow, dynThreshHigh);
			#endif
			uint16_t countStrongEdge = 0;
			uint16_t countWeakEdge = 0;
			// Evaluate thresholding suppressed image gradient
			for (Index xCol = 0; xCol < cols; ++xCol ) { // scan image rows in left col top to bottom, col moves left to right over image
				for (Index yRow = 0; yRow < rows; ++yRow) {
					//Based on the strength of the gradient in the bmax suppressed image, assign
					if (suppressed(yRow, xCol) >= dynThreshHigh) {
						// Strong edge candidate
						edgeCandidates.emplace_back(xCol, yRow);
						if(returnMaskRequest) { //if edge mask requested, update passed matrix
							(*imageEdgeMaskReturn)(yRow, xCol) = static_cast<typename EdgeMaskType::Scalar>(setStrongMaskTo);  // assign strongCandidate
						}
						#ifdef _DEBUG_MTXCV
							//printf("STRONG:(%u,%u) ", static_cast<uint16_t>(xCol), static_cast<uint16_t>(yRow));
						#endif
						countStrongEdge++;
					} else if (suppressed(yRow, xCol) >= dynThreshLow && suppressed(yRow, xCol) < dynThreshHigh) {
						// Weak edge candidate
						edgeCandidates.emplace_back(xCol, yRow);
						if(returnMaskRequest) { //if edge mask requested, update passed matrix
							(*imageEdgeMaskReturn)(yRow, xCol) = static_cast<typename EdgeMaskType::Scalar>(setWeakMaskTo);  // assign weakCandidate
						}
						#ifdef _DEBUG_MTXCV
							//printf("WEAK:(%u,%u) ", static_cast<uint16_t>(xCol), static_cast<uint16_t>(yRow));
						#endif
						countWeakEdge++;
					} else {// Pixel below min edge threshold

					}
				}
			}
			#ifdef _DEBUG_MTXCV
				printf("\n\r[Info] @cannyEdgeDetection: countStrongEdge[%u] countWeakEdge[%u].\n\r", countStrongEdge, countWeakEdge);
				// printf("\n\rsuppressed:\n\r");
				// printMatrix(suppressed);
			#endif
			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
		}

		/*Validate coordinate point is within image boundary via Index
		Parameters:
		- xCoord: x of (x,y) coordinate to evaluate
		- yCoord: y of (x,y) coordinate to evaluate
		- xCols: total number of x columns in image to be bound within
		- yRows: total number of y rows in image to be bound within
		- xStartIdx: Select a different x column index to be bound within (default is column index 0)
		- yStartIdx: Select a different y row index to be bound within (default is row index 0)
		Returns: True if coord is within image, false otherwise */
		template<typename CoordType> //allows both Index and int, int coords allowed for potential negative coord bound checks
		static inline bool isValidImageCoord(const CoordType& xCoord, const CoordType& yCoord, const Index& xCols, const Index& yRows, const Index xStartIdx = 0, const Index yStartIdx = 0) {
			return (static_cast<int>(xCoord) >= xStartIdx && static_cast<int>(xCoord) < xCols && static_cast<int>(yCoord) >= yStartIdx && static_cast<int>(yCoord) < yRows); //boolean return indicating valid coordinate in image
		}
		// Overloaded function to accept Vector2i coord
		static inline bool isValidImageCoord(const Vector2i& coord, const Index& xCols, const Index& yRows, const Index xStartIdx = 0, const Index yStartIdx = 0) {
			return isValidImageCoord(coord.x(), coord.y(), xCols, yRows, xStartIdx, yStartIdx); //expand coord to x, y and return evaluation result
		}

		/*Function takes a (x,y) Coord and finds the EdgePoint object in edgeCandidates with the same referenced coordinate using binary search
		NOTE: This approach only possibe due edgeCandidates vector being ordered ascending lowest to highest coord due cannyEdgeDetection scans image row left to right, col top to bottom;
		the assessed Coord(x,y) is therefore reversed in the lamda function to (y,x) to account for the yRow being the major incrementing range (yRow is incremented each time xCol reaches max)
		Parameters:
		- edgeCandidates: Vector of edge candidates to find the targetCoord coordinate within (ORDER OF THE LOADED VECTOR, AND BINARY SEARCH CONDITION IS CRITICAL FOR CORRECT FUNCTIONALITY)
		- targetCoord: Coord pair to find in provided vector of edge candidates
		Returns: Pointer to identified Edge point in edgeCandidates, or nullptr if nothing found */
		static EdgePoint* getEdgePointRefFromCoord(const vector<EdgePoint>& edgeCandidates, const Vector2i& targetCoord) { // binarySearch()
			#ifdef _DEBUG_MTXCV
				//printf("@getEdgePointRefFromCoord finding (%d, %d).\n\r", targetCoord.first, targetCoord.second);
			#endif
			// Perform binary search to find the first EdgePoint element in edgeCandidates where EdgePoint.coord is NOT less than targetCoord
			// Ordered List Only: for reference: https://en.cppreference.com/w/cpp/algorithm/lower_bound
			auto itr = std::lower_bound(edgeCandidates.begin(), edgeCandidates.end(), targetCoord, // range to asses and input value
													[](const EdgePoint& edge, const Vector2i& coord) { // binary logic lamda function
														#ifdef _DEBUG_MTXCV
															//printf("peek [%d, %d] < [%d, %d].\n\r", get_x(edge.coord), get_y(edge.coord), get_x(coord), get_y(coord));
														#endif
														// ### TBD check if causing issue with rotated image as search depends on the order image was scanned ###
														// Compare component-wise
														return (edge.coord.x() < coord.x() || (edge.coord.x() == coord.x() && edge.coord.y() < coord.y())); // (xCol, yRow) Col is major increment (left col rows top to bottom, col left to right)

													});
			// itr is iterator pointing to lower_bound result; check an element was found and has the same Coord as targetCoord
			if (itr != edgeCandidates.end() && itr->coord == targetCoord) {
				//dereference itr pointer into EdgePoint object, taking its address is a pointer to the actual object, cast to remove const assignment returned by lower_bound
				return const_cast<EdgePoint*>(&(*itr)); // return pointer to underlying EdgePoint in edgeCandidates with matching targetCoord
			} else {
				return nullptr; // Coord not found in edgeCandidates, return nullptr
			}
		}

		/*Function extracts an NxN patch from an image centered around a given image coordinate.
		Parameters:
		- image: Input image to extract the patch from
		- kernel_N: square NxN patch size to extract, must be odd numbered
		- coord.x_coord: x coordinate col index in image to centre extracted patch
		- coord.y_coord: y coordinate row index in image to centre extracted patch
		- patchReturn: returned NxN patch from image at given coordinates
		Returns: executed function state */
		template<typename MatrixType>
		static FnState getImagePatchFromCoord(const MatrixType& image, const int& kernel_N, const Vector2i& coord, MatrixXf& patchReturn) {
			const int rows = static_cast<int>(image.rows());
			const int cols = static_cast<int>(image.cols());
			// Check if kernel size is odd and non-zero
			if (kernel_N % 2 == 0 || kernel_N <= 0 || rows < kernel_N || cols < kernel_N) {
				#ifdef _DEBUG_MTXCV
					//std::cerr << "[Err] @getEigenMatrixFromThermalArray: Size mismatch between provided array and matrix rows*cols.\n\r";
					printf("[Err] @getImagePatchFromCoord: Kernel size must be an odd, non-zero integer smaller than the reference image.\n\r");
				#endif
				patchReturn.setZero();
				return FnState::PROCESSING_ERR; // Return an empty matrix
			}
			// calc kernel half size
			const int kernelPadding = (kernel_N - 1) / 2;

			// init returned image patch
			patchReturn.resize(kernel_N, kernel_N);
			patchReturn.setZero();

			// extract x, y image coordinates from coord
			const int x_coord = coord.x();
			const int y_coord = coord.y();

			//printf("coord (x_col, y_row):(%d, %d)\n\r", x_coord, y_coord);

			// Clamp the patch coordinates within the image bounds
			const int x_start = std::max(0, x_coord - kernelPadding);
			const int y_start = std::max(0, y_coord - kernelPadding);
			const int x_end = std::min(cols - 1, x_coord + kernelPadding);
			const int y_end = std::min(rows - 1, y_coord + kernelPadding);

			// Calculate the indices for the block operation
			const int x_offset = x_coord - x_start;
			const int y_offset = y_coord - y_start;
			const int x_length = x_end - x_start + 1;
			const int y_length = y_end - y_start + 1;

			// Extract the patch from the image using block operations to ensure the image is referenced only within its border;
			// sections of the patch outside the image are left blank and filled with mirroring below
			patchReturn.block(kernelPadding - y_offset, kernelPadding - x_offset, y_length, x_length) = image.template cast<float>().block(y_start, x_start, y_length, x_length);


			// Construct the out of bounds pixels in the patch by mirroring the bound pixels if the patch exceeds the image edge
			if (x_length != kernel_N || y_length != kernel_N) {
				const int x_leftMirror = std::max(0, kernelPadding - x_coord); // result > 0 indicating num cols in patch outside left of image boundary
				const int x_rightMirror = std::max(0, (x_coord + kernelPadding + 1) - cols); // result > 0 indicating num cols in patch outside right of image boundary
				const int y_topMirror = std::max(0, kernelPadding - y_coord); // result > 0 indicating num rows in patch outside top of image boundary
				const int y_bottomMirror = std::max(0, (y_coord + kernelPadding + 1) - rows); // result > 0 indicating num rows in patch outside bottom of image boundary

				if(x_rightMirror > 0) {
					//mirror patch pixels to fill areas of the patch which exceed the image boundary to the right side
					patchReturn.middleCols(kernel_N - x_rightMirror, x_rightMirror) = patchReturn.middleCols(kernel_N - x_rightMirror - x_rightMirror, x_rightMirror).rowwise().reverse();
				}
				if (x_leftMirror > 0) {
					//mirror patch pixels to fill areas of the patch which exceed the image boundary to the left side
					patchReturn.middleCols(0, x_leftMirror) = patchReturn.middleCols(x_leftMirror, x_leftMirror).rowwise().reverse();
				}
				if (y_bottomMirror > 0) {
					//mirror patch pixels to fill areas of the patch which exceed the bottom of the image boundary
					patchReturn.middleRows(kernel_N - y_bottomMirror, y_bottomMirror) = patchReturn.middleRows(kernel_N - y_bottomMirror - y_bottomMirror, y_bottomMirror).colwise().reverse();
				}
				if (y_topMirror > 0) {
					//mirror patch pixels to fill areas of the patch which exceed the top of the image boundary
					patchReturn.middleRows(0, y_topMirror) = patchReturn.middleRows(y_topMirror, y_topMirror).colwise().reverse();
				}
			}
			return FnState::OK;
		}

		/* Function preforms breadth first search (BFS) from a provided seedCoord and returns the sequence of edges associated with it.
		Equations adapted from: https://en.wikipedia.org/wiki/Breadth-first_search; https://cp-algorithms.com/graph/breadth-first-search.html
		NOTE: A sequence is only started when the value in imageEdgeMask at seedCoord is >= thresholdHigh, subsequent points in the sequence must remain >= thresholdLow to be included in sequence;
		Ensure seedCoord is a valid coord in imageEdgeMask before calling this function (this function validates searched coordinate subsequent to seedCoord)
		Parameters:
		- imageEdgeMask: Matrix image edge mask showing identfied candidate edges to be evaluated
		- edgeCandidates: Vector of edge candidate coordinates to evaluate, pointers to the relevent objects in this vector are what are returned when an edge sequence is identified.
		- seedCoord: Starting coordinate to begin BFS from, this coordinate must be a Strong candidate (> thresholdHigh) to begin a sequence.
		- visited: boolean vector index matched to image pixels (via [y * cols + x]), tracks coordinate points that have already been evaluated  / identified in a sequence
		- sequenceLen: Returned variable indicating number of edges making up the identified sequence, or 0 if no sequence found from seedCoord
		- thresholdLow: An edge coord being evaluated must have a value above thresholdLow to be included in the currently evalutated sequence (the sequence must remain above this value to continue)
		- thresholdHigh: A sequence cannot be started (ie seedCoord) unless its value is above thresholdHigh
		Returns: Vector of EdgePoint pointers to the objects in edgeCandidates which form an edge sequence identified by BFS and originating from seedCoord; coordinates already flagged as visited in the visited vector are excluded from current BFS search */
		template<typename MatrixType>
		static FnState breadthFirstSearch(const MatrixType& imageEdgeMask, const vector<EdgePoint>& edgeCandidates, const Vector2i& seedCoord, vector<bool>& visited, vector<EdgePoint*>& edgeSequenceReturn, const uint8_t& thresholdLow = 128, const uint8_t& thresholdHigh = 255) {
			// Direction vectors for Moore neighborhood; current ordering traverses the neighboring pixels clockwise starting from left (west) of pxProbe
			constexpr int dx[] = {-1, -1, 0, 1, 1, 1, 0, -1}; // 8 cardinal directions and diagonals to neighboring pixels
			constexpr int dy[] = {0, -1, -1, -1, 0, 1, 1, 1};

			FnState fnState = FnState::INITIALISED;
			const Index rows = imageEdgeMask.rows();
			const Index cols = imageEdgeMask.cols();

			//reinitialise and clear passed sequence
			edgeSequenceReturn.clear();
			int lastEdgeConfidence = static_cast<int>(imageEdgeMask(seedCoord.y(), seedCoord.x())); // must be strong candidate to start contour

			//Validate starting coord is a strong candidate
			if(lastEdgeConfidence >= thresholdHigh) {
				queue<Vector2i> queue; // Queue for BFS // std::queue is last-in first-out
				queue.push(seedCoord); // Push seedCoord to queue
				visited[seedCoord.y() * cols + seedCoord.x()] = true; // Mark seedCoord as visited by BFS

				while (!queue.empty()) {
					Vector2i pxProbe = queue.front(); // Get the front pixel from the queue
					queue.pop(); // Remove the front pixel from the queue

					// As pxProbe coord is generated from imageEdgeMask values, we use binarySearch to find the pointer to the EdgePoint in edgeCandidates with the same Coord reference
					EdgePoint* edgePointPtr = getEdgePointRefFromCoord(edgeCandidates, pxProbe);
					if(edgePointPtr != nullptr){ // check relevent reference pointer has been found (should always find as edgeCandidates and imageEdgeMask are generated together in cannyEdgeDetection)
						edgeSequenceReturn.push_back(edgePointPtr); // add pointer to the referenced EdgePoint to the returned contour sequence
					} else {
						#ifdef _DEBUG_MTXCV
							//std::cerr << "[Err] @breadthFirstSearch: binarySearch for edge coordinate in edgeCandidates returned nullptr, continuing with invalid linked edge.\n\r";
							printf("[Err] @breadthFirstSearch: binarySearch for edge coordinate in edgeCandidates returned nullptr, continuing with invalid linked edge.\n\r");
							fnState = FnState::PROCESSING_ERR;
						#endif
					}
					// Explore neighbors using 8 surrounding px (ie Moore neighborhood)
					for (int i = 0; i < 8; ++i) { //starting 0 indicates pixel left (west) of seedCoord, search continues CW
						int x_probe = pxProbe.x() + dx[i]; //int to allow Index invalid negative coords when pxProbe is a border pixel, filtered with isValidImageCoord
						int y_probe = pxProbe.y() + dy[i];
						// Check if neighbor is within image bounds and not visited
						if (isValidImageCoord(x_probe, y_probe, cols, rows) && !visited[y_probe * cols + x_probe]) {
							int edgeConfidence = static_cast<int>(imageEdgeMask(y_probe, x_probe)); //extract intensity of detected neighbor edge in mask at coordinate
							// Check if neighbor pixel intensity is above threshold low
							if (edgeConfidence >= thresholdLow) { // candidates after seed must remain above weak threshold to continue contour
								Vector2i pxProbe_boundary = {x_probe, y_probe}; // Neighbor pixel coordinate
								queue.push(pxProbe_boundary); // Add neighbor pixel to queue for further exploration
								//int lastEdgeConfidence = edgeConfidence; // TBD not currently used
							}
							visited[y_probe * cols + x_probe] = true; // Mark neighbor as visited
						}
					}
				}
			} else {
				// sequence not started as seedCoord is not a strong candidate, return empty
				fnState = FnState::PROCESSING_WARN; // TBD if warning is suitable for this type of return given it is expected for some pixels
			}
			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
		}

		/*Function to perform edge tracking of identified candidate edge points in image; this function calls breadthFirstSearch() and returns all sequences found in the image as a Contour object
		Parameters:
		- imageEdgeMask: Edge mask of image indicating strong and weak edge candidates in image
		- edgeCandidates: Vector of edge point candidate coordinates to asses, these coords should match the imageEdgeMask and are the objects pointed to in the edge sequences returned in Contour
		- minSequenceLength: Set the minimum sequence of edge points required to qualify being added to edgeSequenceListReturn
		- thresholdLow: An edge coord being evaluated must have a value above thresholdLow to be included in the currently evalutated sequence (the sequence must remain above this value to continue)
		- thresholdHigh: A sequence cannot be started (ie seedCoord) unless its value is above thresholdHigh
		Returns: Contour is a vector of a vector of EdgePoint pointers, ie a vector of edge sequences found in the provided imageEdgeMask from the potential edgeCandidates */
		template<typename MatrixType>
		static FnState edgeTrackingByHysteresis(const MatrixType& imageEdgeMask, const vector<EdgePoint>& edgeCandidates, vector<EdgePoint*>& edgeSequenceListReturn, const size_t& minSequenceLength, const uint8_t& lowThreshold = 128, const uint8_t& highThreshold = 255) {
			FnState fnState = FnState::INITIALISED;
			const Index rows = imageEdgeMask.rows();
			const Index cols = imageEdgeMask.cols();
			#ifdef _DEBUG_MTXCV
				printf("[Info] @edgeTrackingByHysteresis: Processing %d edge candidates in %dx%d edge mask.\n\r", static_cast<int>(edgeCandidates.size()), static_cast<int>(cols), static_cast<int>(rows));
			#endif
			if (rows == 0 || cols == 0) {
				#ifdef _DEBUG_MTXCV
					printf("[Err] @edgeTrackingByHysteresis: Invalid image provided with a 0 dimension.\n\r");
				#endif
				return FnState::PROCESSING_ERR; // return empty EdgePoints vector indicating error
			}
			//re-init contour list to empty
			//edgeSequenceListReturn.linkedEdges.clear();
			edgeSequenceListReturn.clear();
			vector<EdgePoint*> edgeSequence; //init edge sequence to be populated by BFS, these returned sequences are added to edgeSequenceListReturn
			vector<bool> visited(rows * cols, false); // init bool vector to track which pixels have been assessed

			// for each identified edge coordinate in vector
			for (const EdgePoint& seedCoord : edgeCandidates) {
				// Check if the candidate pixel is within the image bounds
				if (!isValidImageCoord(seedCoord.coord.x(), seedCoord.coord.y(), cols, rows)) {
					#ifdef _DEBUG_MTXCV // should never occur
						//std::cerr << "[Warn] @edgeTrackingByHysteresis: Edge candidate coord is out of image bounds, continuing next.\n\r";
						printf("[Warn] @edgeTrackingByHysteresis: Edge candidate coord [%d,%d] is out of image bounds, continuing next.\n\r", static_cast<int>(seedCoord.coord.x()), static_cast<int>(seedCoord.coord.y()));
					#endif
					fnState = evalFnStates(fnState, FnState::PROCESSING_WARN);
					continue; //skip current point coordinate and proceed to next edgePoint
				}
				// Check bool vector to see if the candidate pixel has already been visited
				size_t idxVisited = seedCoord.coord.y() * cols + seedCoord.coord.x(); // calculate index in vector corresponding to pixel coord
				if (!visited[idxVisited]) {
					// Perform breadth-first search (BFS) from the current candidate
					// Returned edgeSequence is sequence of connected edge points found from seedCoord
					breadthFirstSearch(imageEdgeMask, edgeCandidates, seedCoord.coord, visited, edgeSequence, lowThreshold, highThreshold);
					// If connected edge point found, add them to returned list of edge lists
					if (!edgeSequence.empty() && edgeSequence.size() >= minSequenceLength) {
						//fnState = FnState::OK; //if at least one sequence identified in image, return OK unless other err occured
						// If a valid sequence of edges was found, add it to the returned Contour list
						//edgeSequenceListReturn.linkedEdges.push_back(edgeSequence);
						edgeSequenceListReturn = edgeSequence;
						return evalFnStates(fnState, FnState::OK);
					}
				}
			}
			return evalFnStates(fnState, FnState::OK); //return error/warn if occured, otherwise OK
		}

//		/* Function to dynamically calculate Otsu threshold for canny edge detection; image must have values mapped to 0-255 range before calling function;
//		returned threshHighReturn is the UPPER thresholdHigh pixel intensity identified for canny edge detection, generally recommended canny thresholdLow as 1/2 or 1/3 of thresholdHigh.
//		Equations and approach adapted from: https://en.wikipedia.org/wiki/Otsu%27s_method
//		Parameters:
//		- image: Input image to be assessed
//		- threshHighReturn: returned canny threshold high recommendation
//		Returns: NA */
//		template<typename MatrixType>
//		static FnState calculateOtsuThreshold(const MatrixType& image, float& threshHighReturn) {
//			const Index rows = image.rows();
//			const Index cols = image.cols();
//			// Check if the image is empty
//			if (rows == 0 || cols == 0) {
//				#ifdef _DEBUG_MTXCV
//					//std::cerr << "[Err] @calculateOtsuThreshold: Empty image or invalid intensity range.\n\r";
//					printf("[Err] @calculateOtsuThreshold: Empty image provided.\n\r");
//				#endif
//				threshHighReturn = 0.0f;
//				return FnState::PROCESSING_ERR; // on err return min value as default
//			}
//			//init array to store calculated histogram
//			Array256byte histogram = Array256byte::Zero();
//			getImageHistogram(image, histogram); // Calculate the histogram of the image
//
//			// init vars to track probabilities and cumulative sums
//			float sum = 0.0f; //cumulative sum for probability calc
//			float sumB = 0.0f;
//			int weightBackground = 0;
//			int weightForeground = static_cast<int>(rows * cols); //image total pixels
//			float varMax = -1.0f;
//			// for each intensity value in image between [minValue, maxValue]
//			for (size_t t = 0; t <= static_cast<size_t>(UINT8_MAX); ++t) {
//				weightBackground += histogram(t); //increment background weight by num pixels at current intensity
//				weightForeground -= histogram(t); //subtract num pixels at current intensity from foreground weight
//				if (weightBackground == 0 || weightForeground == 0) {
//					continue; //skip to next intensity to avoid div 0
//				}
//				sum += t * histogram(t);
//				sumB += t * histogram(t);
//
//				// calculate average cumulative intensity foreground and background
//				float meanBackground = sumB / weightBackground;
//				float meanForeground = (sum - sumB) / weightForeground;
//				// Calculate between-class variance
//				float varBetween = weightBackground * weightForeground * (meanBackground - meanForeground) * (meanBackground - meanForeground);
//
//				// Update maximum between-class variance and threshold
//				if (varBetween > varMax) {
//					varMax = varBetween;
//					threshHighReturn = static_cast<float>(t); //record current intensity level
//				}
//			}
//			// normalise threshold to ratio of largest intensity in image; this is done to align with current implementation of canny
//			//threshHighReturn = threshHighReturn / static_cast<float>(image.maxCoeff());
//			return FnState::OK;
//		}

		//Seperated function to calculate array sum of image patch convolution with zernike mask (approach seperates memory allocation to save FLASH space at runtime)
		// Zernike static masks adapted from https://github.com/swq123459/ZernikeMoment_SubPixel_Detection
		static float getZernikeVal_11_real(MatrixXf currImagePatch) {
			const Eigen::Matrix<float, 7, 7> ZERNIKE_MASK_11_REAL = (Eigen::Matrix<float, 7, 7>() <<
				0.f,      -0.015f,  -0.019f,  0.f, 0.019f,  0.015f,  0.f,
				-0.0224f, -0.0466f, -0.0233f, 0.f, 0.0233f, 0.0466f, 0.0224f,
				-0.0573f, -0.0466f, -0.0233f, 0.f, 0.0233f, 0.0466f, 0.0573f,
				-0.069f,  -0.0466f, -0.0233f, 0.f, 0.0233f, 0.0466f, 0.069f,
				-0.0573f, -0.0466f, -0.0233f, 0.f, 0.0233f, 0.0466f, 0.0573f,
				-0.0224f, -0.0466f, -0.0233f, 0.f, 0.0233f, 0.0466f, 0.0224f,
				0.f,      -0.015f,  -0.019f,  0.f, 0.019f,  0.015f,  0.f).finished();

			MtxCV::convolution(currImagePatch, ZERNIKE_MASK_11_REAL, true);
			return currImagePatch.array().sum();
		}

		//Seperated function to calculate array sum of image patch convolution with zernike mask (approach seperates memory allocation to save FLASH space at runtime)
		static float getZernikeVal_11_imag(MatrixXf currImagePatch) {
			//[NOTE ZERNIKE_MASK_11_IMAG is transpose of ZERNIKE_MASK_11_REAL]
			const Eigen::Matrix<float, 7, 7> ZERNIKE_MASK_11_IMAG = (Eigen::Matrix<float, 7, 7>() <<
				0.f,     -0.0224f, -0.0573f, -0.069f,   -0.0573f, -0.0224f, 0.f,
				-0.015f, -0.0466f, -0.0466f, -0.0466f,  -0.0466f, -0.0466f, -0.015f,
				-0.019f, -0.0233f, -0.0233f, -0.0233f,  -0.0233f, -0.0233f, -0.019f,
				0.f,     0.f,      0.f,      0.f,       0.f,      0.f,      0.f,
				0.019f,  0.0233f,  0.0233f,  0.0233f,   0.0233f,  0.0233f,  0.019f,
				0.015f,  0.0466f,  0.0466f,  0.0466f,   0.0466f,  0.0466f,  0.015f,
				0.f,     0.0224f,  0.0573f,  0.069f,    0.0573f,  0.0224f,  0.f).finished();

			MtxCV::convolution(currImagePatch, ZERNIKE_MASK_11_IMAG, true);
			return currImagePatch.array().sum();
		}

		//Seperated function to calculate array sum of image patch convolution with zernike mask (approach seperates memory allocation to save FLASH space at runtime)
		static float getZernikeVal_20(MatrixXf currImagePatch) {
			const Eigen::Matrix<float, 7, 7> ZERNIKE_MASK_20 = (Eigen::Matrix<float, 7, 7>() <<
				0.f,     0.0225f,  0.0394f,  0.0396f,  0.0394f,  0.0225f,  0.f,
				0.0225f, 0.0271f,  -0.0128f, -0.0261f, -0.0128f, 0.0271f,  0.0225f,
				0.0394f, -0.0128f, -0.0528f, -0.0661f, -0.0528f, -0.0128f, 0.0394f,
				0.0396f, -0.0261f, -0.0661f, -0.0794f, -0.0661f, -0.0261f, 0.0396f,
				0.0394f, -0.0128f, -0.0528f, -0.0661f, -0.0528f, -0.0128f, 0.0394f,
				0.0225f, 0.0271f,  -0.0128f, -0.0261f, -0.0128f, 0.0271f,  0.0225f,
				0.f,     0.0225f,  0.0394f,  0.0396f,  0.0394f,  0.0225f,  0.f).finished();

			MtxCV::convolution(currImagePatch, ZERNIKE_MASK_20, true);
			return currImagePatch.array().sum();
		}

		// Approach adapted from DOI: 10.2514/1.A33692; referenced equations as applicable.
		template<typename MatrixType>
		static FnState getZernikeSubpixelAdjustment(MatrixType& image, const vector<EdgePoint*>& edgePoints) {
		    // init function execution state
			FnState fnState = FnState::INITIALISED;
		    const int kernel_N = 7; //determined by Zernike NxN size

			// Fixed-size matrices
			//constexpr int rows = MatrixType::RowsAtCompileTime;
			//constexpr int cols = MatrixType::ColsAtCompileTime;

		    // image patch under investigation, image is padded with extended border if coord results kernel out of image bound
		    MatrixXf currImagePatch(kernel_N, kernel_N);

		    // w is only used on length calculation and appears as w^2 in each instance, see eq 66
		    const float rampWidthSquared = (0.332f * 0.332f);
		    float psi = 0.0f; // angle between zernike frame rotated to perpendicular to identified edge, and image frame x-axis (to left)
		    float length = 0.0f; //zernike frame distance to edge

		    // Iterate through the edge points in sequence, get image patch around coordinate, and perform convolution with zernike masks
		    for (EdgePoint* point : edgePoints) {
		        int x_edgeCoord = point->coord.x();
		        int y_edgeCoord = point->coord.y();

		        // get patch from image centred around edge coordinate
		        MtxCV::getImagePatchFromCoord(image, kernel_N, point->coord, currImagePatch);
				
				//perform image patch convolutions with zernike moments
		        float zern_11_real = getZernikeVal_11_real(currImagePatch);
				float zern_11_imag = getZernikeVal_11_imag(currImagePatch);
				float zern_20 = getZernikeVal_20(currImagePatch);

		        //rotation angle from perpendicular to edge in zernike frame to x-axis in original frame
		        psi = atan2f(zern_11_imag, zern_11_real); // based on eq.56
		        // capture trig results for psi to avoid repeat calculations below
		        float cos_psi = cos(psi);
		        float sin_psi = sin(psi);

		        //Calculate the edge alignment frame by assessing the horizontal and vertical tilt aberrations
		        float zern_11_edgeAlignedFrame = (zern_11_real * cos_psi) + (zern_11_imag * sin_psi); // based on eq. 60

		        // fit ramp model to edge and calculate magnitude of vector to its location
		        length = (1 - rampWidthSquared - sqrtf(powf(rampWidthSquared - 1, 2) - 2 * rampWidthSquared * (zern_20 / zern_11_edgeAlignedFrame))) / rampWidthSquared; //eq. 66
		        float scalingFactor = (kernel_N * kernel_N * length) / 2.0f; // rescale the distance to the ramp back to pixels coordinates

		        #ifdef _DEBUG_MTXCV
		        	printf("[Info] @getZernikeSubpixelAdjustment: length [%.4f]; scalingFactor [%.4f].\n\r", length, scalingFactor);
				#endif

		        //Calculate and store pixel adjustment in image frame; based on eq. 70
		        point->adjustment.x() = (scalingFactor * cos_psi) + x_edgeCoord;
		        point->adjustment.y() = (scalingFactor * sin_psi) + y_edgeCoord;

		    }

		    return evalFnStates(fnState, FnState::OK);
		}

		//Transform point sequence from pixel space to world space
		static void mapEdgeSequence2WorldFrame(const vector<EdgePoint*>& edgePoints, const Matrix3f& K_inv, const float Z_distance = 1.f) { //const bool mapAdjustedCoord = true,
		    //init return vector of world coords
			//vector<Vector3f> world3DCoords;
		    //vector<Vector2f> world2DCoords;
		    //world2DCoords.reserve(edgePoints.size()); // Reserve space to avoid reallocations
//			const float Z_distance = 1.f;
//			const Matrix3f K_inv = (Eigen::Matrix3f() <<
//					0.0325f,	0, 			-0.5206f,
//					0,			0.0263f, 	-0.3153f,
//					0,			0, 			1.f).finished();
//			const Matrix3f M_inv = (Eigen::Matrix3f() <<
//					-1.f,	0, 		0,
//					0,		-1.f, 	0,
//					0,		0, 		1.f).finished();

			//calculate transformed coordinates
		    for (const auto& edgePoint : edgePoints) {

		    	float u_adj = edgePoint->adjustment.x();
		    	float v_adj = edgePoint->adjustment.y();
		    	float u = static_cast<float>(edgePoint->coord.x());
		    	float v = static_cast<float>(edgePoint->coord.y());

				Vector3f worldCoord3D;
				// ### Transform sub-pixel adjusted coordiante to world space ###
				worldCoord3D.x() = K_inv(0, 0) * u_adj + K_inv(0, 2);
				worldCoord3D.y() = K_inv(1, 1) * v_adj + K_inv(1, 2);
				worldCoord3D.z() = Z_distance;
				// Apply the inverse extrinsic matrix (M_inv) manually
				edgePoint->adjustment.x() = -worldCoord3D.x() * Z_distance;  // M_inv applies -1 to x
				edgePoint->adjustment.y() = -worldCoord3D.y() * Z_distance;  // M_inv applies -1 to y
//				//update coordinate with new transformed world coord
//				edgePoint->adjustment = worldCoord3D.head<2>();

				// ### Transform original edge pixel coordiante to world space ###
				// not the most elegant solution i know...but working is working :D
				worldCoord3D.setZero();
				worldCoord3D.x() = K_inv(0, 0) * u + K_inv(0, 2);
				worldCoord3D.y() = K_inv(1, 1) * v + K_inv(1, 2);
				worldCoord3D.z() = Z_distance;
				// Apply the inverse extrinsic matrix (M_inv) manually
				edgePoint->coord.x() = static_cast<int>(-worldCoord3D.x() * Z_distance);  // M_inv applies -1 to x
				edgePoint->coord.y() = static_cast<int>(-worldCoord3D.y() * Z_distance);  // M_inv applies -1 to y

//				//Convert pixel coordinates to homogeneous coordinates
//				Vector3f pixelHomogeneous(u, v, 1.0f);
//				//Apply inverse intrinsic matrix (K_inv) to get normalised camera coords
//				Vector3f normalisedCoords = K_inv * pixelHomogeneous;
//				// Scale by Z distance
//				Vector3f worldCoord3D = normalisedCoords * Z_distance;
//				worldCoord3D.z() = Z_distance;
//				// Apply the inverse extrinsic matrix (M_inv) to map to the world frame
//				worldCoord3D = M_inv * worldCoord3D;

				//2D WORLD COORDS
				//world2DCoords.push_back(worldCoord2D);
				//3D WORLD COORDS
				//world3DCoords.push_back(worldCoord3D);
		    }

		    //return world2DCoords;
		    //return world3DCoords;
		}


		// Function to fit a conic section to given pixel coordinates using least squares
		// const vector<EdgePoint*> edgePoints
		static FnState getConicParams(const vector<Vector2f>& edgePoints, Matrix<float, 6, 1>& paramsReturn) {
			const float zeroPrecision = 1e-5f; //Set precision for what classifies as 0
			//const float residualErrorTolerance = 1e-1f; //residual error should be less than this precision
			FnState fnState = FnState::INITIALISED; //Track function execution state

			size_t n = edgePoints.size();
			MatrixXf M(n, 6);  // Matrix to store the coefficients for each point

			// Construct matrix M where each row corresponds to the equation:
			// Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
			for (size_t i = 0; i < n; ++i) {
				float x = edgePoints[i].x();
				float y = edgePoints[i].y();

				//Populate conic matrix
				M(i, 0) = x * x;  // Coefficient for x^2
				M(i, 1) = x * y;  // Coefficient for xy
				M(i, 2) = y * y;  // Coefficient for y^2
				M(i, 3) = x;      // Coefficient for x
				M(i, 4) = y;      // Coefficient for y
				M(i, 5) = 1;      // const
			}

			/*
			MatrixXf MtM = M.transpose() * M;
			SelfAdjointEigenSolver<MatrixXf> eigSolver(MtM);
			if (eigSolver.info() != Success) {
				paramsReturn.setZero();
				return evalFnStates(fnState, FnState::PROCESSING_WARN); //flag function warning and return
			}
			// Get the eigenvector corresponding to the smallest eigenvalue
			VectorXf singularValues = eigSolver.eigenvalues();
			MatrixXf eigenVectors = eigSolver.eigenvectors();
			*/	//<< when using SelfAdjointEigenSolver
//
			// Perform Singular Value Decomposition (SVD) on matrix M; find least-squares solution
			JacobiSVD<MatrixXf> svd(M, Eigen::ComputeFullV); //<< when using SVD ComputeFullV
			//JacobiSVD<MatrixXf> svd(M, Eigen::ComputeThinV); //<< when using SVD ComputeThinV

			int idx_smallestSingularValue;
			//load identified index of smallest singular value to idx_smallestSingularValue to extract last column later
			//float smallestSingularValue = singularValues.minCoeff(&idx_smallestSingularValue); //<< when using SelfAdjointEigenSolver
			float smallestSingularValue = svd.singularValues().minCoeff(&idx_smallestSingularValue); //<< when using SVD

			// Check if the smallest singular value is close to zero, ie indicating poor fit to data
			if (smallestSingularValue < zeroPrecision) {
				#ifdef _DEBUG_MTXCV
					printf("[Warn] @getConicParams: Poor conditioning of matrix system M to edge data. Smallest singular value: %f < %.1E.\n\r", smallestSingularValue, zeroPrecision);
				#endif
				paramsReturn.setZero(); //zero returned params vector
				return evalFnStates(fnState, FnState::PROCESSING_WARN); //flag function warning and return
			}

			// The parameters of the conic section are the indexed column of matrixV, corresponding to the smallest singular values
			//paramsReturn = eigenVectors.col(idx_smallestSingularValue); //<< when using SelfAdjointEigenSolver
			paramsReturn = svd.matrixV().col(idx_smallestSingularValue); //<< when using SVD
//
			// Check for reasonable parameters (eg not all zeros)
			if (paramsReturn.norm() < zeroPrecision) {
				#ifdef _DEBUG_MTXCV
					printf("[Warn] @getConicParams: Fitted parameters are near-zero, indicating a possible failure.\n\r");
				#endif
				paramsReturn.setZero(); //zero returned params vector
				return evalFnStates(fnState, FnState::PROCESSING_WARN); //flag function warning and return
			}
			return evalFnStates(fnState, FnState::OK); //return function execution result
		}

		// Function to classify the conic section and compute its properties
		static void classifyConic(const Matrix<float, 6, 1>& params) {
			float A = params(0);
			float B = params(1);
			float C = params(2);
			float D = params(3);
			float E = params(4);
			float F = params(5);
			// // Direct conic quadratic matrix form
			// Matrix<float, 3, 3> conicMatrix;
			// conicMatrix << A,      	B / 2.f, 	D / 2.f,
			// 			B / 2.f, 	C,      	E / 2.f,
			// 			D / 2.f, 	E / 2.f, 	F;

			// float conicDeterEigen = conicMatrix.determinant();

			//Calculate the determinant of the conic section from its conic quadratic matrix
			//float conicDeter = A*((C*F) - (E/2.f*E/2.f)) - (B/2.f)*((B/2.f*F) - (E/2.f * D/2.f)) + (D/2.f)*((B/2.f*E/2.f) - (C*D/2.f));
			// Calculate the conic discriminant to classify the conic section
			float conicDiscr = B * B - 4 * A * C;
			// Calculate the eccentricity
			float eccentricity = computeEccentricity(params); //, &conicDeter

			//#ifdef _DEBUG_MTXCV
				printf("---\n\rConic eq: %fx^2 + %fxy + %fy^2 + %fx + %fy + %f = 0\n\r", A, B, C, D, E, F);
				//printf(">> determinant: %f; discriminant: %f\n\r>> eccentricity: %f\n\r", conicDeter, conicDiscr, eccentricity);
				printf(">> eccentricity: %f\n\r", eccentricity);
			//#endif

				// Classify the conic
			if(!isnan(eccentricity)) {
				if (conicDiscr < 0) {
					if(A == C && B == 0){
						//Identified circle
						printf("The conic is a circle.\n\r");
					} else {
						//Identified ellipse
						printf("The conic is an ellipse.\n\r");
					}
				} else if (conicDiscr == 0) {
					//Identified parabola
					printf("The conic is a parabola.\n\r");
				} else {
					//Identified hyperbola
					printf("The conic is a hyperbola.\n\r");
				}
			} else {
				//Undefined conic
				printf("Undefined conic.\n\r");
				return;
			}

			const float beta_coneSurfaceDeg = 64.3f; // 66.07f; ########  TEMP TESTING HARD CODED beta angle ie theta_hoz as 700km orbitl altotude, tbd change to pass var into function #######

			float beta_coneSurfaceRad = beta_coneSurfaceDeg * (static_cast<float>(M_PI) / 180.0f);  // Convert degrees to radians
			float alpha_cuttingPlaneRad = std::acos(eccentricity * std::cos(beta_coneSurfaceRad));
			float bore2NadirVectorRad = (static_cast<float>(M_PI) / 2) - alpha_cuttingPlaneRad;

			// Calculate the center of the conic section by solving partial derivatives
			float x0 = 0;
			float y0 = 0;
			//avoid div 0 in case of circle
			if (conicDiscr == 0 && A == C && B == 0) {
				// circle
				x0 = -D / (2 * A);
				y0 = -E / (2 * A);
			} else {
				x0 = (2 * C * D - B * E) / conicDiscr;
				y0 = (2 * A * E - B * D) / conicDiscr;
			}

			// Calculate the rotation angle of the conic section within the image plane (phi)
			float phi = 0.5f * std::atan2(B, A - C);

			//#ifdef _DEBUG_MTXCV
				printf("beta: %f; alpha: %f; bore2Nadir: %f \n\r", beta_coneSurfaceRad * (180.f / M_PI), alpha_cuttingPlaneRad * (180.f / M_PI), bore2NadirVectorRad * (180.f / M_PI));
				printf("Conic Center: (%f, %f)\n\r", x0, y0);
				printf("Conic Rotation (deg): %f\n\r ", phi * (180.f / M_PI));
			//#endif
		}

		// Function to compute the eccentricity of the provided conic parameters conic[A, B, C, D, E, F]
		static float computeEccentricity(const Matrix<float, 6, 1>& conic) { //, float *conicDeterReturn = nullptr
			// --- Initial function ---
			/*
			float A = conic(0);
			float B = conic(1);
			float C = conic(2);
			float D = conic(3);
			float E = conic(4);
			float F = conic(5);
			//determinant of the conic section from its conic quadratic matrix
			float conicDeter = A*((C*F) - (E*E)/4.f) - (B/2.f)*((F*B/2.f) - (E/2.f * D/2.f)) + (D/2.f)*((B/2.f*E/2.f) - (C*D/2.f));
			//determinent factor (greek symbol Eta) to adjust sign in eccentricity calc
			float determinantFactor = 0.f;
			determinantFactor = (conicDeter < 0 ? 1.f : -1.f);
			float stagingCalc = sqrtf((A-C)*(A-C) + B*B); //stage the repeated calc
			return sqrtf((2*stagingCalc) / (determinantFactor*(A+C) + stagingCalc));
			*/
			//const float deter_zeroTolerance = 1e-6f; //when the conic determinant is 0 the conic is degenerate
		    // Create the conic 3x3 matrix
		    Matrix3f conicMatrix;
//		    conicMatrix << A, B / 2, D / 2,
//		                   B / 2, C, E / 2,
//		                   D / 2, E / 2, F;
		    conicMatrix << 	conic(0), 		conic(1)/2.f, 	conic(3)/2.f,
		    				conic(1)/2.f, 	conic(2),		conic(4)/2.f,
							conic(3)/2.f, 	conic(4)/2.f, 	conic(5);
			// Compute the determinant of the conic matrix
			float conicDeter = conicMatrix.determinant();
//			if(conicDeterReturn != nullptr) {
//				*conicDeterReturn = conicDeter; //assign value in pointer if it was passed to function
//			}
			// --- Reduced required function copying at expense of readability :(, below is equivelant to above ---
			//determinant of the conic section from its conic quadratic matrix
			//float conicDeter = conic(0)*((conic(2)*conic(5)) - (conic(4)*conic(4))/4.f) - (conic(1)/2.f)*((conic(5)*conic(1)/2.f) - (conic(4)/2.f * conic(3)/2.f)) + (conic(3)/2.f)*((conic(1)/2.f*conic(4)/2.f) - (conic(2)*conic(3)/2.f));
			//determinent factor (greek symbol Eta) to adjust sign in eccentricity calc
			float determinantFactor = 0.f;
			determinantFactor = (conicDeter < 0 ? 1.f : -1.f);
			float stagingCalc = sqrtf((conic(0)-conic(2))*(conic(0)-conic(2)) + conic(1)*conic(1)); //stage the repeated calc

			return sqrtf((2*stagingCalc) / (determinantFactor*(conic(0)+conic(2)) + stagingCalc));
			//return (std::abs(conicDeter) <= deter_zeroTolerance ? 0.f : sqrtf((2*stagingCalc) / (determinantFactor*(conic(0)+conic(2)) + stagingCalc)) );
		}

		// Function to compute the algebraic distance of a point to the conic
		static float computeDistanceFromConic(const Matrix<float, 6, 1>& conic, const Vector2f& pt) {
			/*	float A = conic(0);
				float B = conic(1);
				float C = conic(2);
				float D = conic(3);
				float E = conic(4);
				float F = conic(5);*/
			//Equivelant to: Ax^2 + Bxy + Cy^2 + Dx + Ey + F = result
			float result = conic(0) * pt.x() * pt.x() + conic(1) * pt.x() * pt.y() +
							conic(2) * pt.y() * pt.y() + conic(3) * pt.x() +
							conic(4) * pt.y() + conic(5);
			return std::abs(result);  // absolute distance, expectation is close to 0 for good fitting model
		}

		// Linear Congruential Generator (LCG); NOTE modulus to bind number to range should be done within calling function
		// adapted from https://en.wikipedia.org/wiki/Linear_congruential_generator
		static unsigned int lcg_rand(unsigned int& state) {
		    // From X_n+1 = (multiplier * X_n + increment)
			// coefficients selected are credited to Numerical Recipes ranqd1, Chapter 7.1, §An Even Quicker Generator, Eq. 7.1.6 parameters from Knuth and H. W. Lewis
			state = 1664525 * state + 1013904223;  // LCG formula
		    return state;
		}

		// Function to randomly sample n points from coord vector
		static vector<Vector2f> randomSample(const vector<EdgePoint*>& points, const int& numSamples, unsigned int& seed) {
			vector<Vector2f> sample;
			if (numSamples > static_cast<int>(points.size())) {
				return sample; // Return an empty vector if n exceeds the number of points
			}
			vector<int> indices(points.size());
			std::iota(indices.begin(), indices.end(), 0);  // Fill indices with 0, 1, ..., points.size()-1

			// Shuffle indices using LCG-based random number generation
			for (size_t i = 0; i < indices.size(); ++i) {
				std::swap(indices[i], indices[lcg_rand(seed) % indices.size()]); //generate random number and bind to size of
			}

			// Pick the first n elements from the shuffled indices
			for (int i = 0; i < numSamples; ++i) {
				sample.push_back(points[indices[i]]->adjustment);
			}

			return sample;
		}

		// RANdom SAmple Consensus (RANSAC) function to fit a conic using a projective discriminant constraint
		// approach adapted from https://en.wikipedia.org/wiki/Random_sample_consensus
		static FnState ransacConicFitting(const vector<EdgePoint*>& points, Matrix<float, 6, 1>& best_conic, const float eccentricity_minThreshold, const float eccentricity_maxThreshold, const int max_iterations, const float inlier_threshold, unsigned int& seed) {
			const size_t numPoints = points.size();
			FnState fnState = FnState::INITIALISED; //Track function execution state

			if(numPoints <= 5) {
				return evalFnStates(fnState, FnState::PROCESSING_WARN);
			}

			// If numPoints is less than x then use numPoints samples, otherwise use 20% of the total samples or x samples, whichever is bigger
			const int numSamples = std::max(std::min(6, static_cast<int>(numPoints)), static_cast<int>(numPoints * 0.2f)); // RECOMMEND MIN 6 POINTS

			//#ifdef _DEBUG_MTXCV
				printf("[INFO] @ransacConicFitting: Testing %u models each with [%i] random samples of [%u] provided coordinates.\n\r", max_iterations, numSamples, numPoints);
			//#endif

			int max_inliers = 0; //track the max number of inliers found across iterations
			int poorModelFitment_count = 0; //TEMP FOR TESTING, CAN REMOVE
			int candidateModels_count = 0; //TEMP FOR TESTING, CAN REMOVE
			int bestModels_count = 0; //TEMP FOR TESTING, CAN REMOVE
			Matrix<float, 6, 1> candidate_conic;

			for (int i = 0; i < max_iterations; ++i) {
				// Randomly select n points
				vector<Vector2f> selected = randomSample(points, numSamples, seed);
				seed++; //increment seed within function to randomise next selection (Note: seed is already randomised after call within randomSample and lcg_rand

				// ## Print sample points ## 
				// printf("Itteration %u of %u\n\r", i, max_iterations);
				// for (const auto& testP : selected) {
				// 	printf("(%.2f,%.2f),", testP.x(), testP.y());
				// }
				// printf("\n\r");

				// Fit a conic to selected sample points
				candidate_conic.setZero();
				FnState fnState_fitConic = getConicParams(selected, candidate_conic);

				// Check if a model has been successfully fit to the sample data
				if(fnState_fitConic == FnState::OK) {
					// Calculate eccentricity of the conic model (ecc)
					float ecc = computeEccentricity(candidate_conic);
					//Validate eccentricity and skip to next set of samples if not within threshold
					if(isnan(ecc) || (ecc < eccentricity_minThreshold || ecc > eccentricity_maxThreshold)) {
						//#ifdef _DEBUG_MTXCV
							//printf("[%u of %u] Invalid eccentricity %f is outside min threshold %f.\n\r", i + 1, max_iterations, ecc, eccentricity_minThreshold);
						//#endif
						poorModelFitment_count++;
						continue;
					}

					// Count inliers (points close to the candidate conic)
					int inliers_count = 0;
					float inlier_remainderSum = 0;
					for (const auto& pt : points) {
						//expectation is that conicEqRemainder "== 0"
						float conicEqRemainder = computeDistanceFromConic(candidate_conic, pt->adjustment);
						if (conicEqRemainder < inlier_threshold) {
							++inliers_count;
							inlier_remainderSum += conicEqRemainder;
						}
					}

					// Update the best conic if the current one has more inliers
					if (inliers_count > max_inliers) {
						best_conic = candidate_conic;
						max_inliers = inliers_count;
						bestModels_count++;
						#ifdef _DEBUG_MTXCV
							printf("[%u of %u] Found best current candidate; eccentricity = %f\n\r", i + 1, max_iterations, ecc);
							printf("  >Conic eq def: %fx^2 + %fxy + %fy^2 + %fx + %fy + %f = %f\n\r", best_conic(0), best_conic(1), best_conic(2), best_conic(3), best_conic(4), best_conic(5), (inlier_remainderSum/inliers_count));
						#endif

						//exit loop if all points are inliers to reduce unnecessary calcs; greater than just for safety
						if(max_inliers >= static_cast<int>(numPoints)){
							i = max_iterations; //force loop exit
						}
					}
					candidateModels_count++;
				} else {
					//printf("[%u of %u] Poor fitment of conic function to sample points.\n\r", i + 1, max_iterations);
					poorModelFitment_count++;
				}
			}
			#ifdef _DEBUG_MTXCV
				printf("[%u] Candidate models evaluated.\n\r", candidateModels_count);
				printf("[%u] Models poorly conditioned to sample points.\n\r", poorModelFitment_count);
			#endif
			if(max_inliers > 0) {
				fnState = evalFnStates(fnState, FnState::OK); //flag function indicating candidate model found
				//#ifdef _DEBUG_MTXCV
					printf("{%u} Identified best models, with final inlier count [%u of %u].\n\r", bestModels_count, max_inliers, numPoints);
					//Print edge points
					for (const auto& pt : points) {
						printf("(%f,%f),", pt->adjustment.x(), pt->adjustment.y());
					}
					printf("\n\r");
				//#endif
			} else {
				fnState = evalFnStates(fnState, FnState::PROCESSING_WARN); //flag function warning, no valid model
				//#ifdef _DEBUG_MTXCV
					printf("{%u} Models found that fit the data within inlier threshold.\n\r", bestModels_count);
				//#endif
			}
			return fnState;
		}

	}; //END MtxCV CLASS DEFINITION


#endif /* INC_MTXCV_H_ */
