.PHONY: all clean

all: tts_service_pb2.py tts_service_pb2_grpc.py __init__.py

clean:
	rm -f *.py

tts_service_pb2.py tts_service_pb2_grpc.py: tts_service.proto
	python -m grpc_tools.protoc --proto_path=. --python_out=. --grpc_python_out=. tts_service.proto

__init__.py:
	touch __init__.py
