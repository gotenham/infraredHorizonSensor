/*
 * mtxCV.cpp
 *
 *  Created on: Apr 7, 2024
 *      Author: AJ
 */

/* printf REFERENCE:
    %c	character
    %d	decimal (integer) number (base 10)
    %e	exponential floating-point number
    %f	floating-point number
    %i	integer (base 10)
    %o	octal number (base 8)
    %s	a string of characters
    %u	unsigned decimal (integer) number
    %x	number in hexadecimal (base 16)
    %%	print a percent sign
    \%	print a percent sign
NOTE on ImageType template:
template<typename MatrixType>
void exampleFunction(MatrixType& image) {
    //Cast to type associated with MatrixType:
    image(jRow, iCol) = static_cast<typename MatrixType::Scalar>(xyz);

    //Cast MatrixType element to type
    float varFloat = static_cast<float>(image(jRow, iCol));

    //Cast entire image matrix to type
    MatrixXf mtxFloat = image.template cast<float>();

    //convert data type without copying the matrix using map (changes to one matrix are reflected in the other)
    MatrixXf mappedImage(image.rows(), image.cols());
    mappedImage = Map<MatrixXf>(image.data(), image.rows(), image.cols());

// Checking MatrixType data type and structure
    <<Specific DataType>> if constexpr(std::is_same<typename MatrixType::Scalar, float>::value) {} //see std template: is_same<T, T>:std::true_type {};
    <<float, double etc>> if constexpr(std::is_floating_point<typename MatrixType::Scalar>::value) {}
    <<all signed and unsigned integer types>> if constexpr(std::is_integral<typename MatrixType::Scalar>::value) {}
    <<all signed integer types>> if constexpr(std::is_signed<typename MatrixType::Scalar>::value) {}
    <<all unsigned integer types>> if constexpr(std::is_unsigned<typename MatrixType::Scalar>::value) {}
    <<type is Matrix>> if constexpr(Eigen::is_matrix<MatrixType>::value) {}
    <<type is Array>> if constexpr(Eigen::is_array<MatrixType>::value) {}
    <<type is Vector>> if constexpr(Eigen::is_vector<MatrixType>::value) {}

 Use constexpr variable declaration to flag value to be computed at compile time.
}

+++++++++++++ EXAMPLE IMG ROTATIONS +++++++++++++ //
    //FLIP VERTICAL
    thermalImg.rowwise().reverseInPlace();

    //FLIP HORIZONTAL
    thermalImg.colwise().reverseInPlace();

    //ROTATE CCW 90 degrees
    thermalImg.transposeInPlace();
    thermalImg.colwise().reverseInPlace();

    //ROTATE CW 90 degrees
    thermalImg.transposeInPlace();
    thermalImg.rowwise().reverseInPlace();
+++++++++++++ NEXT SECTION +++++++++++++ //
*/
	#include "mtxCV.h"

	// Example data structure for MLX90640 sensor image output for testing
//	const float thermalImageArr[768] = {0.497f, 0.f, 0.1f, 0.1f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
//	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 6.497f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.624f, 0.f, 0.f, 6.497f, 0.f, 0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 0.f,
//	1.624f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 6.497f, 0.f,
//	0.f, 1.624f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 6.497f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
//	0.f, 6.497f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
//	0.f, 0.f, 0.f, 0.f, 0.f, 6.497f, 0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 6.497f, 0.f, 0.f, 0.f, 0.f, 0.f,
//	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
//	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
//	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.624f, 1.624f, 42.229f, 42.229f, 102.325f, 102.325f, 139.682f, 139.682f, 159.172f, 159.172f, 155.924f, 155.924f, 134.809f, 134.809f, 108.822f, 108.822f, 79.586f, 79.586f, 48.726f, 48.726f,
//	0.f, 6.497f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.624f, 1.624f, 42.229f, 42.229f, 102.325f, 102.325f, 139.682f, 139.682f, 159.172f, 159.172f, 155.924f, 155.924f, 134.809f, 134.809f, 108.822f, 108.822f, 79.586f, 79.586f, 48.726f, 48.726f,
//	0.f, 0.f, 1.624f, 0.f, 0.f, 0.f, 4.873f, 4.873f, 84.459f, 84.459f, 190.032f, 190.032f, 255.f, 255.f, 245.255f, 245.255f, 222.516f, 222.516f, 191.656f, 191.656f, 159.172f, 159.172f, 134.809f, 134.809f, 113.694f, 113.694f, 92.58f, 92.58f, 73.089f, 73.089f, 51.975f, 51.975f,
//	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 4.873f, 4.873f, 84.459f, 84.459f, 190.032f, 190.032f, 255.f, 255.f, 245.255f, 245.255f, 222.516f, 222.516f, 191.656f, 191.656f, 159.172f, 159.172f, 134.809f, 134.809f, 113.694f, 113.694f, 92.58f, 92.58f, 73.089f, 73.089f, 51.975f, 51.975f,
//	0.f, 0.f, 16.242f, 16.242f, 100.701f, 100.701f, 198.153f, 198.153f, 235.51f, 235.51f, 216.019f, 216.019f, 162.42f, 162.42f, 147.803f, 147.803f, 154.299f, 154.299f, 128.312f, 128.312f, 100.701f, 100.701f, 92.58f, 92.58f, 87.707f, 87.707f, 66.592f, 66.592f, 45.478f, 45.478f, 42.229f, 42.229f,
//	0.f, 0.f, 16.242f, 16.242f, 100.701f, 100.701f, 198.153f, 198.153f, 235.51f, 235.51f, 216.019f, 216.019f, 162.42f, 162.42f, 147.803f, 147.803f, 154.299f, 154.299f, 128.312f, 128.312f, 100.701f, 100.701f, 92.58f, 92.58f, 87.707f, 87.707f, 66.592f, 66.592f, 45.478f, 45.478f, 42.229f, 42.229f,
//	50.35f, 50.35f, 115.318f, 115.318f, 141.306f, 141.306f, 175.414f, 175.414f, 157.548f, 157.548f, 128.312f, 128.312f, 103.949f, 103.949f, 99.076f, 99.076f, 81.21f, 81.21f, 69.841f, 69.841f, 63.344f, 63.344f, 47.102f, 47.102f, 37.357f, 37.357f, 32.484f, 32.484f, 22.739f, 22.739f, 19.49f, 19.49f,
//	50.35f, 50.35f, 115.318f, 115.318f, 141.306f, 141.306f, 175.414f, 175.414f, 157.548f, 157.548f, 128.312f, 128.312f, 103.949f, 103.949f, 99.076f, 99.076f, 81.21f, 81.21f, 69.841f, 69.841f, 63.344f, 63.344f, 47.102f, 47.102f, 37.357f, 37.357f, 32.484f, 32.484f, 22.739f, 22.739f, 19.49f, 19.49f,
//	50.35f, 50.35f, 82.834f, 82.834f, 134.809f, 134.809f, 105.573f, 105.573f, 79.586f, 79.586f, 79.586f, 79.586f, 66.592f, 66.592f, 45.478f, 45.478f, 73.089f, 73.089f, 71.465f, 71.465f, 38.981f, 38.981f, 29.236f, 29.236f, 21.115f, 21.115f, 16.242f, 16.242f, 9.745f, 9.745f, 4.873f, 4.873f,
//	50.35f, 50.35f, 82.834f, 82.834f, 134.809f, 134.809f, 105.573f, 105.573f, 79.586f, 79.586f, 79.586f, 79.586f, 66.592f, 66.592f, 45.478f, 45.478f, 73.089f, 73.089f, 71.465f, 71.465f, 38.981f, 38.981f, 29.236f, 29.236f, 21.115f, 21.115f, 16.242f, 16.242f, 9.745f, 9.745f, 4.873f, 4.873f,
//	32.484f, 32.484f, 55.223f, 55.223f, 60.096f, 60.096f, 38.981f, 38.981f, 42.229f, 42.229f, 56.847f, 56.847f, 45.478f, 45.478f, 29.236f, 29.236f, 40.605f, 40.605f, 29.236f, 29.236f, 21.115f, 21.115f, 17.866f, 17.866f, 14.618f, 14.618f, 8.121f, 8.121f, 3.248f, 3.248f, 6.497f, 6.497f,
//	32.484f, 32.484f, 55.223f, 55.223f, 60.096f, 60.096f, 38.981f, 38.981f, 42.229f, 42.229f, 56.847f, 56.847f, 45.478f, 45.478f, 29.236f, 29.236f, 40.605f, 40.605f, 29.236f, 29.236f, 21.115f, 21.115f, 17.866f, 17.866f, 14.618f, 14.618f, 8.121f, 8.121f, 3.248f, 3.248f, 6.497f, 6.497f,
//	14.618f, 14.618f, 11.369f, 11.369f, 12.994f, 12.994f, 19.49f, 19.49f, 32.484f, 32.484f, 30.86f, 30.86f, 24.363f, 24.363f, 21.115f, 21.115f, 24.363f, 24.363f, 17.866f, 17.866f, 8.121f, 8.121f, 8.121f, 8.121f, 4.873f, 4.873f, 1.624f, 1.624f, 1.624f, 1.624f, 3.248f, 3.248f,
//	14.618f, 14.618f, 11.369f, 11.369f, 12.994f, 12.994f, 19.49f, 19.49f, 32.484f, 32.484f, 30.86f, 30.86f, 24.363f, 24.363f, 21.115f, 21.115f, 24.363f, 24.363f, 17.866f, 17.866f, 8.121f, 8.121f, 8.121f, 8.121f, 4.873f, 4.873f, 1.624f, 1.624f, 1.624f, 1.624f, 3.248f, 3.248f,
//	6.497f, 6.497f, 4.873f, 4.873f, 4.873f, 4.873f, 6.497f, 6.497f, 8.121f, 8.121f, 11.369f, 11.369f, 9.745f, 9.745f, 8.121f, 8.121f, 6.497f, 6.497f, 6.497f, 6.497f, 3.248f, 3.248f, 3.248f, 3.248f, 4.873f, 4.873f, 3.248f, 3.248f, 1.624f, 1.624f, 0.f, 0.f,
//	6.497f, 6.497f, 4.873f, 4.873f, 4.873f, 4.873f, 6.497f, 6.497f, 8.121f, 8.121f, 11.369f, 11.369f, 9.745f, 9.745f, 8.121f, 8.121f, 6.497f, 6.497f, 6.497f, 6.497f, 3.248f, 3.248f, 3.248f, 3.248f, 4.873f, 4.873f, 3.248f, 3.248f, 1.624f, 1.624f, 0.f, 0.f};


