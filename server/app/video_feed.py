from flask import render_template

@app.route('/video_feed')
def video_feed():
    # Simulate video feed data
    frame = '<img src="https://picsum.photos/200/300">'
    return render_template('video_feed.html', frame=frame)
