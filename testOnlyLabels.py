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
            print(label.description)
            #print(label.description)
            if ('Tin can' in label.description) or ('Canteen'in label.description) or ('Plastic bottle'in label.description) or ('Flask'in label.description) or ('Water'in label.description) or ('Water bottle'in label.description) or ('Bottle' in label.description) or ('Cylinder' in label.description) or ('Beverage Can' in label.description) or ('Aluminum Can' in label.description):
                print('Tin can: Grasp 1')
                return 'Grasp 1'
            elif 'Stemware' in label.description:
                print('Stemware: Grasp 2')
                return 'Grasp 2'
            elif ('Cup'  in label.description) or ('Cofee cup'  in label.description):
                print('Cup: Grasp 3')
                return 'Grasp 3'
        print('nothing detected')
        return 'NULL'
            

while True:
    # The name of the image file to annotate
    directory = 'C:/Users/Melvin/source/repos/kapper24/P4_project/images'
    dirs = os.listdir(directory)
    if dirs.__sizeof__()>41:
        filename = dirs[0]
        print(filename)
        imagePath = os.path.join(directory, filename)
        time.sleep(0.1)
        #Detect objects and send to txt file
        write = open('C:/Users/Melvin/source/repos/kapper24/P4_project/ReadfromPython.txt','w')
        write.write(detectObjects(imagePath))
        write.close()
        time.sleep(0.1)
        #detectObjects(imagePath)
        for filename in dirs:
            if os.path.isfile(imagePath) or os.path.islink(imagePath):
                os.unlink(imagePath)
                    