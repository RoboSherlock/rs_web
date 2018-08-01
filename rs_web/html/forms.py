from flask_wtf import FlaskForm
from wtforms import IntegerField, SubmitField, BooleanField
from wtforms import SelectField


class ScenesForm(FlaskForm):
    timestamp = BooleanField('Timestamp')
    from_time = IntegerField('From')
    to_time = IntegerField('To')
    object_id = IntegerField('Id')
    object_c = BooleanField('Object')
    filter = SubmitField('Filter')
    export = SubmitField('Export')


class HypothesisForm(FlaskForm):
    timestamp = BooleanField('Timestamp')
    annotator = BooleanField('Annotator')
    comparison = SelectField('Compare', choices=[('1', '<='), ('2', '=>')])
    filter = SubmitField('Filter')
    export = SubmitField('Export')
