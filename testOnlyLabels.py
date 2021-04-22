import io
import os
import time
# Imports the Google Cloud client library

def detectObjects(path):
    from google.cloud import vision
    
    
    # Instantiates a client
    client = vision.ImageAnnotatorClient()
    
    
    
    # Loads the image into memory
    with io.open(path, 'rb') as image_file:
        content = image_file.read()
        
        image = vision.types.Image(content=content)
        
        # Performs label detection on the image file
        response = client.label_detection(image=image)
        labels = response.label_annotations
        
        #print('Labels:')
        for label in labels:
            #print(label.description)
            if 'Tin can' in label.description:
                print('Tin can: Grasp 1')
                return 'Grasp 1'
            elif 'Stemware' in label.description:
                print('Stemware: Grasp 2')
                return 'Grasp 2'
            elif 'Cup' in label.description:
                print('Cup: Grasp 3')
                return 'Grasp 3'
            else:
                print("nothing detected")
                return 'NULL'
            

while True:
    # The name of the image file to annotate
    directory = 'C:/Users/Melvin/Documents/GUI_test/images'
    dirs = os.listdir(directory)
    if dirs.__sizeof__()>41:
        filename = dirs[0]
        print(filename)
        imagePath = os.path.join(directory, filename)
        time.sleep(0.1)
        #Detect objects and send to txt file
        write = open('Read.txt','w')
        write.write(detectObjects(imagePath))
        write.close()
        
        #detectObjects(imagePath)
        for filename in dirs:
            if os.path.isfile(imagePath) or os.path.islink(imagePath):
                os.unlink(imagePath)
                    