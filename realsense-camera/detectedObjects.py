class DetectedObject:
    def __init__(self, label='obj', confidence=None, bounding_box=None):
        self.label = label  # Object label or class name
        self.confidence = confidence  # Confidence score of detection
        self.bounding_box = bounding_box  # Bounding box coordinates (x, y, width, height)
        # the bounding box parameters are as follows: x - top left corner, y - top left corner, width, height

    def __str__(self):
        return f'{self.label} (Confidence: {self.confidence:.2f})'