#!/bin/bash

set -ex

# Taken from https://gist.github.com/lelegard/6a428f67ee08e86d0c2f1af3f4a633d0

# Customize these lines with your repository and credentials:
REPO=https://api.github.com/repos/$GITHUB_REPOSITORY
GITHUB_USER=${GITHUB_REPOSITORY%/*} 


# Number of most recent versions to keep for each artifact:
KEEP=0

# A shortcut to call GitHub API.
ghapi() { curl --silent --location "$@"; }

# A temporary file which receives HTTP response headers.
TMPFILE=/tmp/tmp.$$

# An associative array, key: artifact name, value: number of artifacts of that name.
declare -A ARTCOUNT

# Process all artifacts on this repository, loop on returned "pages".
URL=$REPO/actions/artifacts
while [[ -n "$URL" ]]; do

    # Get current page, get response headers in a temporary file.
    JSON=$(ghapi --dump-header $TMPFILE "$URL")

    # Get URL of next page. Will be empty if we are at the last page.
    URL=$(grep '^Link:' "$TMPFILE" | tr ',' '\n' | grep 'rel="next"' | head -1 | sed -e 's/.*<//' -e 's/>.*//')
    rm -f $TMPFILE

    # Number of artifacts on this page:
    COUNT=$(( $(jq <<<"$JSON" -r '.artifacts | length') ))

    # Loop on all artifacts on this page.
    for ((i=0; i < COUNT; i++)); do

        # Get name of artifact and count instances of this name.
        name=$(jq <<<"$JSON" -r ".artifacts[$i].name?")
        ARTCOUNT[$name]=$(( $(( ${ARTCOUNT[$name]} )) + 1))

        # Check if we must delete this one.
        if [[ ${ARTCOUNT[$name]} -gt $KEEP ]]; then
            id=$(jq <<<"$JSON" -r ".artifacts[$i].id?")
            size=$(( $(jq <<<"$JSON" -r ".artifacts[$i].size_in_bytes?") ))
            printf "Deleting '%s' #%d, %'d bytes\n" "$name" ${ARTCOUNT[$name]} $size
            curl --user "$GITHUB_USER":"$GITHUB_TOKEN" \
                 --request DELETE \
                 --url "$REPO"/actions/artifacts/"$id" 
        fi
    done
done
