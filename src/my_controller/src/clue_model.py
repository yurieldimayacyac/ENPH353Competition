import torch
import torch.nn as nn
import torch.nn.functional as F

class ClueBoardDetector(nn.Module):
  def __init__(self):
    super(ClueBoardDetector, self).__init__()

    # FLATTENING THE DATA
    num_channels = 32

    # Input dimensions of image (should be for each letter)
    self.letter_width = 32
    self.letter_height = 32

    # Scanning
    self.conv_layer1 = nn.Conv2d(1,num_channels,kernel_size=3,padding=1)
    self.pool_layer1 = nn.MaxPool2d(2,2)
    self.conv_layer2 = nn.Conv2d(num_channels, num_channels*2, kernel_size=3,padding=1)
    self.pool_layer2 = nn.MaxPool2d(2,2)

    self.dropout = nn.Dropout(0.5) # dropout for training

    num_hidden_units = 128
    num_classes = 36

    ppool_width = self.letter_width // 4 # post-pooling width
    ppool_height = self.letter_height // 4 # post-pooling height
    flattened_size = ppool_width * ppool_height * num_channels * 2

    # FC LAYERS
    self.fc_layer1 = nn.Linear(flattened_size, num_hidden_units)
    self.fc_layer2 = nn.Linear(num_hidden_units, num_classes)

  def forward(self, data):
    # First Convolutional Layer
    data = self.conv_layer1(data)
    data = torch.relu(data) # noise filtering
    data = self.pool_layer1(data)

    # Second Convolutional Lyaer
    data = self.conv_layer2(data)
    data = torch.relu(data)
    data = self.pool_layer2(data)

    # Flatten Data
    data = data.view(data.size(0), -1)

    # First Linear Layer
    data = torch.relu(self.fc_layer1(data))
    data = self.dropout(data)

    data = self.fc_layer2(data)

    return data