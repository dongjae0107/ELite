echo Start downloading the ParkingLot dataset

mkdir -p data/parkinglot
cd data/parkinglot

# 01.zip
gdown https://drive.google.com/uc?id=1jZJQQKLAFIvPAIda4a0LsfNrUVQsqVBK
unzip -q 01.zip -d 01
rm 01.zip

# 02.zip
gdown https://drive.google.com/uc?id=1WpqRhpLyCIUKhd_aFwm687POBhzHiXM6
unzip -q 02.zip -d 02
rm 02.zip

# # 03.zip
# gdown https://drive.google.com/uc?id=1LixqhVSpaU9svrY4nzKk4gCU1hEd3bnC
# unzip -q 03.zip -d 03
# rm 03.zip

# # 04.zip
# gdown https://drive.google.com/uc?id=1A5C5Iv79HHAapmoi6gIA5SfF0d7HJEvd
# unzip 04.zip
# rm 04.zip

# # 05.zip
# gdown https://drive.google.com/uc?id=1pO-a_NgQS9-yXpH6ro9CBapSk1RnHVBO
# unzip 05.zip
# rm 05.zip

# # 06.zip
# gdown https://drive.google.com/uc?id=1LMqgBoPnb9C9cY35qy-dlsiVKKNSb0xv
# unzip 06.zip
# rm 06.zip

cd ../..

echo Finished downloading the ParkingLot dataset