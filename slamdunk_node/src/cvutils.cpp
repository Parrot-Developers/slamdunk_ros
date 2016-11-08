/*
 * Copyright (c) 2016 Parrot S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "cvutils.hpp"

#include <opencv2/imgproc/imgproc.hpp>

cv::Mat yuvToRgb(kalamos::YuvData const& yuv, unsigned int cropWidth, unsigned int cropHeight)
{
    if (yuv[0]->cols < cropWidth)
        cropWidth = yuv[0]->cols;
    if (yuv[0]->rows < cropHeight)
        cropHeight = yuv[0]->rows;

    cv::Rect roi(yuv[0]->cols / 2 - cropWidth / 2,
                 yuv[0]->rows / 2 - cropHeight / 2,
                 cropWidth, cropHeight);

    return yuvToRgb(yuv, roi);
}

cv::Mat yuvToRgb(kalamos::YuvData const& yuv, cv::Rect roi)
{
    // Fix vconcat assert (src[i].cols == src[0].cols)
    if (roi.width % 2)
        roi.width += (2 - roi.width % 2);
    // Fix reshape assert (total elements divisible by new row count)
    if (roi.height % 4)
        roi.height += 4 - (roi.height % 4);

    cv::Rect roiC(roi.x / 2, roi.y / 2, roi.width / 2, roi.height / 2);

    cv::Mat croppedY = (*yuv[0])(roi);
    cv::Mat croppedU = (*yuv[1])(roiC);
    cv::Mat croppedV = (*yuv[2])(roiC);

    // Can't reshape a non-continuous cv::Mat
    if (!croppedU.isContinuous())
        croppedU = croppedU.clone();
    if (!croppedV.isContinuous())
        croppedV = croppedV.clone();

    cv::Mat croppedMatArray[] = {croppedY, croppedU.reshape(0, croppedU.rows / 2), croppedV.reshape(0, croppedV.rows / 2)};

    cv::Mat croppedYUV;
    cv::vconcat(croppedMatArray, 3, croppedYUV);

    cv::Mat rgb;
    cv::cvtColor(croppedYUV, rgb, CV_YUV2RGB_I420);

    return rgb;
}

kalamos::MappedMatSharedPtr yuvToGrayscale(kalamos::YuvData const& yuv, unsigned int cropWidth, unsigned int cropHeight)
{
    if (yuv[0]->cols < cropWidth)
        cropWidth = yuv[0]->cols;
    if (yuv[0]->rows < cropHeight)
        cropHeight = yuv[0]->rows;

    cv::Rect roi(yuv[0]->cols / 2 - cropWidth / 2,
                 yuv[0]->rows / 2 - cropHeight / 2,
                 cropWidth, cropHeight);

    return yuvToGrayscale(yuv, roi);
}

// No cloning -> we have to keep a reference to the original MappedMatSharedPtr
namespace
{
struct DerivedMappedMat
{
    kalamos::MappedMatSharedPtr mapping;
    cv::Mat mat;
};
}

kalamos::MappedMatSharedPtr yuvToGrayscale(kalamos::YuvData const& yuv, cv::Rect roi)
{
    auto derived = std::make_shared<DerivedMappedMat>();
    derived->mapping = yuv[0];
    derived->mat = (*yuv[0])(roi);
    return kalamos::MappedMatSharedPtr(derived, &derived->mat);
}
