{{ fullname | escape | underline}}

.. automodule:: {{ fullname }}

   {% block submodules %}
   {% if modules %}
   .. raw:: html

      <h2>Submodules</h2>

   .. autosummary::
      :recursive:
      :toctree:
   {# Use Jinja sort filter to get a case-insensitive sort order #}
   {% for item in modules|sort %}
      {{item.rpartition('.')[2]}}
   {%- endfor %}

   .. comment to get Sphinx to recognize end of autosummary directive
   {% endif %}
   {% endblock %}

   {% if attributes or functions or classes or exceptions %}
   Summary of Module Members
   =========================

   {% block attributes %}
   {% if attributes %}
   .. rubric:: Module Attributes

   .. autosummary::
   {% for item in attributes %}
      {{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block functions %}
   {% if functions %}
   .. rubric:: Functions

   .. autosummary::
      :nosignatures:
   {% for item in functions %}
      {{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block behaviors %}
   {% if behaviors %}
   .. rubric:: Behaviors

   .. autosummary::
      :nosignatures:
   {% for item in behaviors %}
      {{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block classes %}
   {% if classes %}
   .. rubric:: Classes

   .. autosummary::
      :nosignatures:
   {% for item in classes %}
      {{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block exceptions %}
   {% if exceptions %}
   .. rubric:: Exceptions

   .. autosummary::
      :nosignatures:
   {% for item in exceptions %}
      {{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block scenarios %}
   {% if scenarios %}
   .. rubric:: Scenarios

   .. autosummary::
      :nosignatures:
   {% for item in scenarios %}
      {{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   Member Details
   ==============
   {% endif %}
