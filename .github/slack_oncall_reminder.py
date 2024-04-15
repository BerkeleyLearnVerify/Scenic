import argparse

import requests
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError

# Put users into the dict
def save_users(users_array):
    users = {}
    for user in users_array:
        # NOTE: some apps, slackbots do not have emails to map to
        profile = user["profile"]
        if "email" in profile.keys():
            user_email = profile["email"]
            users[user_email] = user
    return users


def grab_whos_on_call(OPS_GENIE_API_TOKEN, ROTATION_SCHEDULE_ID):
    url = f"https://api.opsgenie.com/v2/schedules/{ROTATION_SCHEDULE_ID}/on-calls"
    headers = {"Authorization": f"GenieKey {OPS_GENIE_API_TOKEN}"}
    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        data = response.json()
    else:
        print(f"Request failed with status code {response.status_code}")
        print("Response content:")
        print(
            response.content.decode("utf-8")
        )  # Decode response content if it's in bytes
    return data["data"]["onCallParticipants"][0]["name"]


def postSlackMessage(client, CHANNEL_ID, OPS_GENIE_API_TOKEN, ROTATION_SCHEDULE_ID):
    try:
        result = client.users_list()
        users = save_users(result["members"])
        on_call = grab_whos_on_call(OPS_GENIE_API_TOKEN, ROTATION_SCHEDULE_ID)
        slack_id = users[on_call]["id"]

        result = client.chat_postMessage(
            channel=CHANNEL_ID,
            text=f"""🛠️ Maintenance On-Call: <@{slack_id}>\n
    📖 <https://https://scenic-lang.atlassian.net/l/cp/cnaQtVXY|On Call Best Practices>
    🔍 <https://scenic-lang.atlassian.net/l/cp/jR0CifEf|Issue Triage Guide>
    📊 <https://scenic-lang.atlassian.net/jira/software/projects/SCENIC/boards/1|Jira Board to monitor active workstreams>
    📋 <https://scenic-lang.atlassian.net/jira/software/projects/SCENIC/boards/1/backlog?epics=visible|Jira Backlog to monitor issues that need triaging>
    🔧 <https://github.com/BerkeleyLearnVerify/Scenic/issues|Scenic GitHub Issues>
    """,
        )
    except SlackApiError as e:
        print(f"SlackAPIError: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Script that notifies on-call rotation daily"
    )
    parser.add_argument("--slack_api_token", required=True, type=str)
    parser.add_argument("--ops_genie_api_token", required=True, type=str)
    args = parser.parse_args()

    SLACK_API_TOKEN = args.slack_api_token
    OPS_GENIE_API_TOKEN = args.ops_genie_api_token
    # NOTE: Feel free to grab the relevant channel ID to post the message to but ensure the App is installed within the channel
    CHANNEL_ID = "C06N9KJHN2J"
    # NOTE: Rotation schedule is grabbed directly from within the OpsGenie
    ROTATION_SCHEDULE_ID = "904cd122-f269-418d-8c29-3e6751716bae"

    client = WebClient(token=SLACK_API_TOKEN)
    postSlackMessage(client, CHANNEL_ID, OPS_GENIE_API_TOKEN, ROTATION_SCHEDULE_ID)
