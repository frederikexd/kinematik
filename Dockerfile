# KinematiK — Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
# Open source. Original author: Frederik Thio, creator of KinematiK.
#
# A frozen environment for the app and the test suite, so a team two
# generations on runs the same Python, the same pinned major versions and the
# same system libraries the published manifests were produced with.
#
#   docker build -t kinematik .
#   docker run --rm -p 8501:8501 kinematik                 # the app, single-user
#   docker run --rm kinematik pytest fsae_suspension/tests -q -p no:cacheprovider \
#       --ignore=fsae_suspension/tests/test_voice_memo.py  # the suite CI runs
FROM python:3.12-slim

# system libraries from fsae_suspension/packages.txt (mesh rendering, xlsx recalc)
RUN apt-get update \
 && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 libreoffice-calc \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt pytest pytest-xdist pypdf

COPY . .
RUN pip install --no-cache-dir -e "./fsae_suspension[excel,cad,pdf,cloud,drive]"

ENV PYTHONPATH=/app/fsae_suspension \
    PYTHONUNBUFFERED=1
EXPOSE 8501
CMD ["streamlit", "run", "fsae_suspension/streamlit_app.py", \
     "--server.address=0.0.0.0", "--server.port=8501", "--server.headless=true"]
