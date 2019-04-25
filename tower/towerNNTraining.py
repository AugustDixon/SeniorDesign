

import numpy as np
import cv2 as cv
import keras
import glob

def main():
    path = '/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower/trainingData/'
    mappingLetterNN = Sequential()
    mappingLetterNN.add(Dense(300, input_shape(self.BLOCK_FACE_Y, self.BLOCK_FACE_X)))
    mappingLetterNN.add(Activation('relu'))
    mappingLetterNN.add(Dense(40))
    mappingLetterNN.add(Activation('relu'))
    mappingLetterNN.add(Dense(6))           #ABCDEF
    mappingLetterNN.add(Activation('softmax'))
    mappingLetterNN.compile(loss='categorical_cross_entropy', optimizer='sgd')
    
    X = Y = []
    
    LowerBlockFace = np.array([0, 0, 115])
    UpperBlockFace = np.array([180, 76, 255])
    
    Apath = path + 'towerA/'
    Bpath = path + 'towerB/'
    Cpath = path + 'towerC/'
    Dpath = path + 'towerD/'
    Epath = path + 'towerE/'
    Fpath = path + 'towerF/'
    
    for filename in glob.glob(os.path.join(Apath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([1, 0, 0, 0, 0, 0])
    #END OF for
    for filename in glob.glob(os.path.join(Bpath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([0, 1, 0, 0, 0, 0])
    #END OF 
    for filename in glob.glob(os.path.join(Cpath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([0, 0, 0, 0, 0, 0])
    #END OF for
    for filename in glob.glob(os.path.join(Dpath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([0, 0, 0, 1, 0, 0])
    #END OF for
    for filename in glob.glob(os.path.join(Epath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([0, 0, 0, 0, 1, 0])
    #END OF for
    for filename in glob.glob(os.path.join(Fpath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([0, 0, 0, 0, 0, 1])
    #END OF for
    
    
    mappingLetterNN.fit(X, Y, epochs=150, batch_size=10, verbose=0)
    
    
    
    model_json = mappingLetterNN.to_json()
    with open(path + "mapping.json", "w") as json_file:
        json_file.write(model_json)
    # serialize weights to HDF5
    mappingLetterNN.save_weights(path + "mapping.h5")
    
#END OF main()


if __name__ == '__main__':
    main()