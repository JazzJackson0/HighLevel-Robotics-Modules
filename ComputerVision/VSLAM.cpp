#include "../include/camera/VSLAM.hpp"
/**
 * NOT FINISHED!!
 * 
 */

vSLAM::vSLAM() {


}

vSLAM::vSLAM(bool stereo) {

    if (stereo) {

    }

    else {


    }

}

cv::Mat vSLAM::BagOfWords(cv::Mat image, cv::Mat img_descriptors,std::vector<cv::KeyPoint> img_keypoints, cv::Ptr<cv::ORB> detector) {

    int clusterSize = 100; // Number of clusters (visual words)

    // 1. Build the visual words vocabulary using k-means clustering
    cv::Ptr<cv::BOWKMeansTrainer> bowTrainer = cv::makePtr<cv::BOWKMeansTrainer>(clusterSize);
    bowTrainer->add(img_descriptors);
    
    // 2. Cluster descriptors to form vocabulary (visual words)
    cv::Mat visual_words = bowTrainer->cluster();

    // 3. Set up BOW descriptor extractor 
    // [FLANN-based matcher]: Match descriptor to a cluster center (word). Used to assign each descriptor to the nearest cluster center (visual word).
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    cv::Ptr<cv::BOWImgDescriptorExtractor> bowDE = cv::makePtr<cv::BOWImgDescriptorExtractor>(detector, matcher);
    bowDE->setVocabulary(visual_words);

    // 4. Compute the BOW descriptor for image
    cv::Mat bowDescriptor; 
    bowDE->compute(image, img_keypoints, bowDescriptor);

    return bowDescriptor; // Histogram
}


double vSLAM::Compare_Histograms(cv::Mat hist1, cv::Mat hist2) {
    
    // Normalize. Ensure comparison not affected by scale
    cv::Mat hist1Norm, hist2Norm;
    normalize(hist1, hist1Norm, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    normalize(hist2, hist2Norm, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // Cosine Similarity: Measure the angle between two histograms (good for sparse vectors). [-1 (completely dissimilar) <-> 1 (exactly similar)]
    double comparison = compareHist(hist1Norm, hist2Norm, cv::HISTCMP_CORREL);
    
    // // Chi-Square: Focus on relative differences (good for normalized histograms) [Emphasizes difference in freq counts btw corresponding bins].
    // double comparison = compareHist(hist1Norm, hist2Norm, cv::HISTCMP_CHISQR);
    
    // // Histogram Intersection: Checks overlap between histograms
    // double comparison = compareHist(hist1Norm, hist2Norm, cv::HISTCMP_INTERSECT);

    // // Bhattacharyya distance
    // double comparison = compareHist(hist1Norm, hist2Norm, cv::HISTCMP_BHATTACHARYYA);

    return comparison;
}

void vSLAM::Optimize() {

    
}



void vSLAM::RunMono(cv::Mat img) {

    mono_odom->Run(img);
}


void vSLAM::RunStereo(cv::Mat left_img, cv::Mat right_img) {

    stereo_odom->Run(left_img, right_img);
}