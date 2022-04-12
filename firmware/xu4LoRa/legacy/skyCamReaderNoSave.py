from datetime import timezone
import time
import os
import datetime
import numpy as np
import pickle
from skimage import io, color
import cv2

from mintsJetson import mintsSkyCamReader as mSCR
from mintsJetson import mintsSensorReader as mSR
from mintsJetson import mintsDefinitions as mD


dataFolder = mD.dataFolder


def main():
    # edited april 16 th
    sensorName   = "SKYCAM_002"
    dateTimeNow  = datetime.datetime.now()
    subFolder    = mSR.getWritePathSnaps(sensorName,dateTimeNow)


    onboardCapture = True
    try:
        currentImage,imagePath =  mSCR.getSnapShotXU4(subFolder)
        start = time.time()
        modelName = 'naiveBayesModel.sav'
        oneDImage, imageShape = mSCR.generateFeatures(currentImage,imagePath)
        print("Loading Classifier")
        loadedModel = pickle.load(open(modelName, 'rb'))
        print("Done Loading")
        predictionBinary,prediction = mSCR.getPredictionMatrix(loadedModel,oneDImage)
        print("Deleting Resulting Images ...")
        binaryImage = mSCR.writeBinaryImageXU4NoSave(predictionBinary,imageShape,imagePath,onboardCapture)
        sensorDictionary  = mSCR.getResultsXU4002(currentImage,binaryImage,predictionBinary,prediction,imagePath,dateTimeNow)
        mSR.sensorFinisher(dateTimeNow,sensorName,sensorDictionary)
        mSCR.timeTaken("Preiction time is ",start)
    except:
        print("TRY AGAIN")


if __name__ == "__main__":
   main()