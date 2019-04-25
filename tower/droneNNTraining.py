
import numpy as np
import cv2 as cv
import keras
import glob

def main():
    path = '/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower/trainingData/'
    droneLetterNN = Sequential()
    droneLetterNN.add(Dense(100, input_shape(self.DRONE_IMAGE_Y, self.DRONE_IMAGE_X)))
    droneLetterNN.add(Activation('relu'))
    droneLetterNN.add(Dense(2))             #[0] - 'B', [1] - 'E'
    droneLetterNN.add(Activation('softmax'))
    droneLetterNN.compile(loss='categorical_cross_entropy', optimizer='sgd')
    
    X = Y = []
    
    LowerBlockFace = np.array([0, 0, 115])
    UpperBlockFace = np.array([180, 76, 255])
    
    Bpath = path + 'droneB/'
    Epath = path + 'droneE/'
    
    for filename in glob.glob(os.path.join(Bpath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([1, 0])
    #END OF for
    for filename in glob.glob(os.path.join(Epath, '*.png')):
        image = cv.imread(filename)
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        if not((image is None) or (image.size == 0)):
            mask = cv.inRange(image, LowerBlockFace, UpperBlockFace)
            X.append(mask)
            Y.append([0, 1])
    #END OF for
    
    
    droneLetterNN.fit(X, Y, epochs=150, batch_size=10, verbose=0)
    
    
    
    
    model_json = droneLetterNN.to_json()
    with open(path + "drone.json", "w") as json_file:
        json_file.write(model_json)
    # serialize weights to HDF5
    droneLetterNN.save_weights(path + "drone.h5")
#END OF main()


if __name__ == '__main__':
    main()