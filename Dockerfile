# Use an official C++ runtime as a parent image
FROM gcc:latest

# Set the working directory in the container to /app
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install any needed packages specified in requirements.txt
RUN apt-get update && apt-get install -y \
    make \
    python3.9 \
    python3-pip

# Make port 80 available to the world outside this container
EXPOSE 80

# Define environment variable
ENV NAME World

# Run make when the container launches
CMD ["make", "all"]